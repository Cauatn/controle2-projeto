"""
Simulação da planta no PC + controlador no Arduino (hardware-in-the-loop).

Uso:
    python hil_planta.py --listar                     # mostra as portas seriais
    python hil_planta.py --porta /dev/cu.usbmodem1101 # roda com o Arduino
    python hil_planta.py --emular                     # sem Arduino (controlador em Python)

A cada período T:
  1. y[k] = C x[k]                      (saída da planta simulada)
  2. envia "r y" ao Arduino, recebe u[k]
  3. x[k+1] = Ad x[k] + Bd u[k]         (planta discretizada por ZOH = exata
                                         para entrada constante entre amostras)

Saídas: figuras/3_resultado_hil.png e resultado_hil.csv
"""
import argparse
import os
import time

import numpy as np
import control as ct

import config as cfg
from projeto_controlador import projetar, coeficientes, malhas_fechadas

AQUI = os.path.dirname(os.path.abspath(__file__))


class ControladorPython:
    """Mesma equação de diferenças do Arduino (para comparação/teste)."""

    def __init__(self, b, a):
        self.b, self.a = b, a
        self.e = np.zeros(len(b))
        self.u = np.zeros(len(a))

    def __call__(self, r, y):
        self.e = np.roll(self.e, 1); self.e[0] = r - y
        self.u = np.roll(self.u, 1)
        u = self.b @ self.e - self.a[1:] @ self.u[1:]
        u = float(np.clip(u, cfg.U_MIN, cfg.U_MAX))
        self.u[0] = u
        return u, 0


class ControladorArduino:
    def __init__(self, porta, baud=115200):
        import serial
        self.ser = serial.Serial(porta, baud, timeout=1.0)
        # abrir a serial reinicia o Arduino: espera a mensagem "PRONTO"
        t0 = time.time()
        while time.time() - t0 < 5:
            if self.ser.readline().decode(errors="ignore").strip() == "PRONTO":
                break
        else:
            print("Aviso: não recebi 'PRONTO' (seguindo mesmo assim).")
        self.ser.reset_input_buffer()
        self.ser.write(b"R\n")
        resp = self.ser.readline().decode(errors="ignore").strip()
        if resp != "OK":
            raise RuntimeError(f"Arduino não respondeu ao reset (recebido: {resp!r})")

    def __call__(self, r, y):
        self.ser.write(f"{r:.6f} {y:.6f}\n".encode())
        resp = self.ser.readline().decode(errors="ignore").split()
        if len(resp) != 2:
            raise RuntimeError(f"Resposta inválida do Arduino: {resp!r}")
        return float(resp[0]), int(resp[1])

    def fechar(self):
        self.ser.close()


def simular(controlador, tempo, degrau, t_degrau, tempo_real):
    G = ct.tf(cfg.NUM_PLANTA, cfg.DEN_PLANTA)
    Gd = ct.ss(ct.sample_system(ct.ss(G), cfg.T, method="zoh"))
    Ad, Bd, Cd = Gd.A, Gd.B, Gd.C

    N = int(round(tempo / cfg.T)) + 1
    x = np.zeros((Ad.shape[0], 1))
    t = np.arange(N) * cfg.T
    r = np.where(t >= t_degrau - 1e-9, degrau, 0.0)
    y = np.zeros(N); u = np.zeros(N); dt = np.zeros(N); atraso = np.zeros(N)

    t_ini = time.perf_counter()
    for k in range(N):
        y[k] = (Cd @ x).item()
        t_env = time.perf_counter()
        u[k], dt[k] = controlador(r[k], y[k])
        atraso[k] = time.perf_counter() - t_env
        x = Ad @ x + Bd * u[k]

        if tempo_real:  # mantém o período T de verdade
            prox = t_ini + (k + 1) * cfg.T
            while time.perf_counter() < prox:
                pass
    return t, r, y, u, dt, atraso


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--porta", help="porta serial do Arduino")
    p.add_argument("--emular", action="store_true", help="roda sem Arduino")
    p.add_argument("--listar", action="store_true", help="lista portas seriais")
    p.add_argument("--tempo", type=float, default=2.0)
    p.add_argument("--degrau", type=float, default=1.0)
    p.add_argument("--t-degrau", type=float, default=0.1)
    args = p.parse_args()

    if args.listar:
        from serial.tools import list_ports
        for porta in list_ports.comports():
            print(f"{porta.device:30s} {porta.description}")
        return

    G, C, Cz, *_ = projetar()
    b, a = coeficientes(Cz)

    if args.emular:
        ctrl, nome = ControladorPython(b, a), "Python (emulado)"
    elif args.porta:
        ctrl, nome = ControladorArduino(args.porta), "Arduino"
    else:
        p.error("informe --porta PORTA ou --emular")

    print(f"Rodando {args.tempo} s com controlador: {nome} ...")
    t, r, y, u, dt, atraso = simular(ctrl, args.tempo, args.degrau, args.t_degrau,
                                     tempo_real=not args.emular)
    if hasattr(ctrl, "fechar"):
        ctrl.fechar()

    # Referência: o mesmo laço com o controlador em Python (resultado "ideal")
    _, _, y_ref, u_ref, _, _ = simular(ControladorPython(b, a), args.tempo, args.degrau,
                                       args.t_degrau, tempo_real=False)

    # Resposta contínua para comparação
    Tc, *_ = malhas_fechadas(G, C, Cz)
    tc = np.linspace(0, args.tempo, 2000)
    uc = np.where(tc >= args.t_degrau, args.degrau, 0.0)
    _, yc = ct.forced_response(Tc, tc, uc)

    # Métricas
    ia = t >= args.t_degrau
    yf = y[ia]; tt = t[ia] - args.t_degrau
    sobress = (yf.max() - args.degrau) / args.degrau * 100
    fora = np.where(np.abs(yf - args.degrau) > 0.02 * abs(args.degrau))[0]
    ts = tt[fora[-1] + 1] if len(fora) and fora[-1] + 1 < len(tt) else float("nan")
    print(f"Sobressinal = {sobress:.2f} %   ts(2%) = {ts:.3f} s   y final = {y[-1]:.4f}")
    print(f"Máx. diferença em relação ao controlador ideal: {np.max(np.abs(y - y_ref)):.2e}")
    if not args.emular:
        print(f"Período medido no Arduino: média {dt[1:].mean()/1000:.3f} ms, "
              f"desvio {dt[1:].std()/1000:.3f} ms")
        print(f"Tempo de ida e volta da serial: média {atraso.mean()*1000:.2f} ms, "
              f"máx {atraso.max()*1000:.2f} ms")

    np.savetxt(os.path.join(AQUI, "resultado_hil.csv"), np.column_stack([t, r, y, u, dt]),
               delimiter=",", header="t,r,y,u,dt_us", comments="")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    a1.plot(tc, yc, "b-", lw=2, alpha=0.6, label="projeto contínuo")
    a1.step(t, y, "r-", where="post", lw=1.3, label=f"HIL — {nome}")
    a1.step(t, r, "k:", where="post", label="referência")
    a1.axhline(args.degrau * (1 + cfg.SOBRESSINAL_MAX), color="gray", ls=":", lw=1)
    a1.set_ylabel("velocidade y"); a1.grid(alpha=0.3); a1.legend(fontsize=8)
    a1.set_title(f"Planta simulada no PC + controlador no {nome.split()[0]}  "
                 f"(Mp = {sobress:.1f} %, ts = {ts:.2f} s)")
    a2.step(t, u, "r-", where="post", label="u enviado pelo controlador")
    a2.set_ylabel("controle u [V]"); a2.set_xlabel("t [s]"); a2.grid(alpha=0.3); a2.legend(fontsize=8)
    fig.tight_layout()
    caminho = os.path.join(AQUI, "figuras", "3_resultado_hil.png")
    fig.savefig(caminho, dpi=150)
    print(f"Figura salva em {caminho}")


if __name__ == "__main__":
    main()
