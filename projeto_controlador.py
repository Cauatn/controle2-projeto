"""
Projeto do controlador pelo Lugar das Raízes + discretização por Tustin.

Uso:
    python projeto_controlador.py

Gera:
    figuras/1_lugar_das_raizes.png
    figuras/2_degrau_continuo_vs_discreto.png
    controlador_arduino/coeficientes.h   (usado pelo Arduino)
"""
import os

import numpy as np
import control as ct

import config as cfg

AQUI = os.path.dirname(os.path.abspath(__file__))


def polo_desejado():
    """Polo dominante desejado s_d = -sigma + j*wd a partir de zeta e sigma."""
    zeta, sigma = cfg.ZETA_PROJ, cfg.SIGMA_PROJ
    wn = sigma / zeta
    wd = wn * np.sqrt(1 - zeta**2)
    return complex(-sigma, wd), wn


def projetar(verbose=False):
    """
    Controlador PI:  C(s) = Kc (s + z0) / s

    1) O integrador (1/s) garante erro nulo ao degrau.
    2) O zero z0 é colocado sobre o polo lento da planta (s = -2), cancelando-o.
       A malha aberta fica  L(s) = 20 Kc / (s (s + 10)), cujo lugar das raízes
       tem a reta vertical em s = -5 -> passa exatamente pelo polo desejado.
    3) Condição de ângulo: confere se s_d pertence ao LGR.
    4) Condição de módulo: Kc = 1 / |L0(s_d)|.
    """
    G = ct.tf(cfg.NUM_PLANTA, cfg.DEN_PLANTA)
    sd, wn = polo_desejado()

    polos_planta = np.sort(np.real(ct.poles(G)))[::-1]  # [-2, -10]
    z0 = -polos_planta[0]                               # zero em -2

    # Malha aberta sem o ganho: L0(s) = (s + z0)/s * G(s)
    L0 = ct.tf([1, z0], [1, 0]) * G
    L0_sd = complex(ct.evalfr(L0, sd))

    angulo = np.degrees(np.angle(L0_sd))
    deficit = (-180.0 - angulo + 180.0) % 360.0 - 180.0  # o que falta para -180°
    Kc = 1.0 / abs(L0_sd)

    C = Kc * ct.tf([1, z0], [1, 0])
    Cz = ct.sample_system(C, cfg.T, method="tustin")

    if verbose:
        print("=" * 60)
        print("PLANTA");  print(G)
        print(f"Polos da planta: {ct.poles(G)}")
        print("=" * 60)
        print("ESPECIFICAÇÕES")
        print(f"  Sobressinal <= {cfg.SOBRESSINAL_MAX*100:.0f} %   ts(2%) <= {cfg.TS_MAX} s   erro nulo ao degrau")
        zmin = -np.log(cfg.SOBRESSINAL_MAX) / np.sqrt(np.pi**2 + np.log(cfg.SOBRESSINAL_MAX)**2)
        print(f"  -> zeta >= {zmin:.3f}   sigma >= {4/cfg.TS_MAX:.2f}")
        print(f"  Escolhido: zeta = {cfg.ZETA_PROJ}, sigma = {cfg.SIGMA_PROJ}, wn = {wn:.3f} rad/s")
        print(f"  Polo desejado s_d = {sd.real:.3f} ± j{sd.imag:.3f}")
        print("=" * 60)
        print("PROJETO PELO LUGAR DAS RAÍZES")
        print(f"  Zero do PI em s = {-z0:.3f} (cancela o polo lento da planta)")
        print(f"  Ângulo de L0(s_d) = {angulo:.2f}°  (deficiência angular = {deficit:.2f}°)")
        if abs(deficit) > 1.0:
            print("  ATENÇÃO: s_d não está no LGR — seria preciso um compensador de avanço.")
        print(f"  Condição de módulo -> Kc = {Kc:.4f}")
        print("  C(s) ="); print(C)
        print("=" * 60)
        print(f"DISCRETIZAÇÃO (Tustin, T = {cfg.T} s)")
        print("  C(z) ="); print(Cz)
        b, a = coeficientes(Cz)
        print("  Equação de diferenças:")
        termos = " ".join(f"{-a[i]:+.6f}·u[k-{i}]" for i in range(1, len(a)))
        termos += " " + " ".join(f"{b[i]:+.6f}·e[k{'-'+str(i) if i else ''}]" for i in range(len(b)))
        print(f"  u[k] = {termos}")

    return G, C, Cz, sd, Kc, z0


def coeficientes(Cz):
    """Retorna (b, a) de C(z) normalizados com a[0] = 1."""
    num, den = ct.tfdata(Cz)
    b = np.atleast_1d(np.squeeze(num[0][0])).astype(float)
    a = np.atleast_1d(np.squeeze(den[0][0])).astype(float)
    b, a = b / a[0], a / a[0]
    if len(b) < len(a):  # alinha (C(z) próprio)
        b = np.concatenate([np.zeros(len(a) - len(b)), b])
    return b, a


def gerar_header(Cz, caminho):
    b, a = coeficientes(Cz)
    fmt = lambda v: ", ".join(f"{x:.9f}f" for x in v)
    texto = f"""// Gerado automaticamente por projeto_controlador.py — NÃO edite à mão.
// C(z) obtido por Tustin com T = {cfg.T} s
// u[k] = sum(B[i]*e[k-i], i=0..NB-1) - sum(A[i]*u[k-i], i=1..NA-1)
#pragma once

#define NB {len(b)}
#define NA {len(a)}
const float B[NB] = {{ {fmt(b)} }};
const float A[NA] = {{ {fmt(a)} }};

const float U_MIN = {cfg.U_MIN:.3f}f;
const float U_MAX = {cfg.U_MAX:.3f}f;
const unsigned long T_US = {int(round(cfg.T * 1e6))}UL;
"""
    with open(caminho, "w", encoding="utf-8") as f:
        f.write(texto)
    print(f"\nArquivo gerado: {caminho}")


def malhas_fechadas(G, C, Cz):
    Tc = ct.feedback(C * G, 1)
    Gd = ct.sample_system(G, cfg.T, method="zoh")
    Td = ct.feedback(Cz * Gd, 1)
    Ud = ct.feedback(Cz, Gd)  # r -> u
    Uc = ct.feedback(C, G)
    return Tc, Td, Uc, Ud


def lugar_das_raizes(L0, ganhos):
    """Calcula o LGR à mão: raízes de den(s) + K num(s) = 0."""
    num, den = ct.tfdata(L0)
    num = np.squeeze(num[0][0]); den = np.squeeze(den[0][0])
    num = np.concatenate([np.zeros(len(den) - len(num)), num])
    return np.array([np.sort_complex(np.roots(den + k * num)) for k in ganhos])


def figuras(G, C, Cz, sd, Kc, z0):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(os.path.join(AQUI, "figuras"), exist_ok=True)

    # ---------- 1) Lugar das raízes ----------
    L0 = ct.tf([1, z0], [1, 0]) * G
    ganhos = np.concatenate([np.linspace(0, 1, 400), np.linspace(1, 30, 1500)])
    raizes = lugar_das_raizes(L0, ganhos)

    fig, ax = plt.subplots(figsize=(7, 5.5))
    for i in range(raizes.shape[1]):
        ax.plot(raizes[:, i].real, raizes[:, i].imag, "b-", lw=1.5)
    ax.plot(np.real(ct.poles(L0)), np.imag(ct.poles(L0)), "kx", ms=10, mew=2, label="polos de malha aberta")
    ax.plot(np.real(ct.zeros(L0)), np.imag(ct.zeros(L0)), "ko", ms=9, mfc="none", mew=2, label="zero do PI")
    mf = lugar_das_raizes(L0, [Kc])[0]
    ax.plot(mf.real, mf.imag, "r*", ms=15, label=f"polos MF (Kc = {Kc:.3f})")

    # Região permitida pelas especificações
    zeta_min = -np.log(cfg.SOBRESSINAL_MAX) / np.sqrt(np.pi**2 + np.log(cfg.SOBRESSINAL_MAX)**2)
    sig_min = 4 / cfg.TS_MAX
    r = 15
    th = np.arccos(zeta_min)
    ax.plot([0, -r * np.cos(th)], [0, r * np.sin(th)], "g--", lw=1)
    ax.plot([0, -r * np.cos(th)], [0, -r * np.sin(th)], "g--", lw=1, label=f"ζ = {zeta_min:.3f}")
    ax.axvline(-sig_min, color="m", ls="--", lw=1, label=f"σ = {sig_min:.1f}")
    ax.axhline(0, color="k", lw=0.5); ax.axvline(0, color="k", lw=0.5)
    ax.set_xlim(-14, 2); ax.set_ylim(-12, 12)
    ax.set_xlabel("Re(s)"); ax.set_ylabel("Im(s)")
    ax.set_title("Lugar das raízes de  Kc·(s+2)/s · G(s)")
    ax.grid(alpha=0.3); ax.legend(loc="lower left", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(AQUI, "figuras", "1_lugar_das_raizes.png"), dpi=150)
    plt.close(fig)

    # ---------- 2) Degrau contínuo vs discreto ----------
    Tc, Td, Uc, Ud = malhas_fechadas(G, C, Cz)
    tf_ = 2.0
    t = np.linspace(0, tf_, 2000)
    _, yc = ct.step_response(Tc, t)
    _, uc = ct.step_response(Uc, t)
    td = np.arange(0, tf_ + cfg.T / 2, cfg.T)
    _, yd = ct.step_response(Td, td)
    _, ud = ct.step_response(Ud, td)

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    a1.plot(t, yc, "b-", lw=2, label="contínuo C(s)")
    a1.step(td, yd, "r--", where="post", lw=1.3, label=f"discreto C(z), Tustin T={cfg.T}s")
    a1.axhline(1 + cfg.SOBRESSINAL_MAX, color="gray", ls=":", label="limite de sobressinal")
    a1.axhspan(0.98, 1.02, color="g", alpha=0.1, label="faixa de 2 %")
    a1.axvline(cfg.TS_MAX, color="m", ls=":", label="ts máx")
    a1.set_ylabel("velocidade y(t)"); a1.grid(alpha=0.3); a1.legend(fontsize=8)
    a1.set_title("Resposta ao degrau em malha fechada")
    a2.plot(t, uc, "b-", lw=2, label="contínuo")
    a2.step(td, ud, "r--", where="post", lw=1.3, label="discreto")
    a2.set_ylabel("controle u(t) [V]"); a2.set_xlabel("t [s]"); a2.grid(alpha=0.3); a2.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(AQUI, "figuras", "2_degrau_continuo_vs_discreto.png"), dpi=150)
    plt.close(fig)

    for nome, sis in [("Contínuo", Tc), ("Discreto", Td)]:
        info = ct.step_info(sis, SettlingTimeThreshold=0.02)
        print(f"{nome:9s}: sobressinal = {info['Overshoot']:.2f} %   ts(2%) = {info['SettlingTime']:.3f} s"
              f"   regime = {info['SteadyStateValue']:.4f}")


if __name__ == "__main__":
    G, C, Cz, sd, Kc, z0 = projetar(verbose=True)
    print("=" * 60)
    print("VERIFICAÇÃO")
    figuras(G, C, Cz, sd, Kc, z0)
    gerar_header(Cz, os.path.join(AQUI, "controlador_arduino", "coeficientes.h"))
    print("Figuras em: figuras/")
