# Projeto Controle 2: Lugar das Raízes + Tustin + Hardware-in-the-loop

- **Planta:** motor DC (velocidade), simulada no PC: `G(s) = 20 / ((s+2)(s+10))`
- **Controlador:** PI projetado pelo lugar das raízes, discretizado por Tustin, rodando no **Arduino**
- **Especificações:** sobressinal ≤ 10 %, ts(2 %) ≤ 1 s, erro nulo ao degrau

```
   PC (hil_planta.py)               Arduino (controlador_arduino.ino)
 ┌──────────────────┐  "r y\n"    ┌───────────────────────────┐
 │ planta G (ZOH)   │ ──────────► │ e = r − y                 │
 │ x ← Ad x + Bd u  │ ◄────────── │ u[k] = u[k-1] + b0 e[k]   │
 └──────────────────┘  "u dt\n"   │        + b1 e[k-1]        │
                                  └───────────────────────────┘
```

## Arquivos

| Arquivo | O que faz |
|---|---|
| `config.py` | planta, especificações, T, saturação (**altere aqui**) |
| `projeto_controlador.py` | lugar das raízes, cálculo de Kc, Tustin, gráficos e `coeficientes.h` |
| `controlador_arduino/` | sketch do Arduino + `coeficientes.h` (gerado) |
| `hil_planta.py` | simula a planta, conversa com o Arduino, plota e salva CSV |
| `figuras/` | figuras para o relatório |

## Como rodar

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt

# 1) Projeto (gera figuras e coeficientes.h)
.venv/bin/python projeto_controlador.py

# 2) Teste sem Arduino
.venv/bin/python hil_planta.py --emular

# 3) Grave controlador_arduino/controlador_arduino.ino pela Arduino IDE
#    (feche o Monitor Serial depois de gravar!)

# 4) Rode com o Arduino
.venv/bin/python hil_planta.py --listar
.venv/bin/python hil_planta.py --porta /dev/cu.usbmodemXXXX
```

## Resumo do projeto

1. **Especificações → polos desejados.**
   Mp ≤ 10 % exige ζ ≥ 0,591, e ts ≤ 1 s exige σ ≥ 4.
   Escolhemos ζ = 0,65 e σ = 5, ou seja, **s_d = −5 ± j5,85**.
   A folga em ζ compensa o atraso que a amostragem introduz.
2. **Estrutura do controlador.** Erro nulo ao degrau pede integrador, então usamos um PI: `C(s) = Kc (s+2)/s`.
   O zero em −2 cancela o polo lento da planta, e a malha aberta fica `L(s) = 20 Kc / (s (s+10))`.
3. **Condição de ângulo.** ∠L(s_d) = −180°: s_d está no LGR, que é a reta vertical em s = −5.
4. **Condição de módulo.** `Kc = |s_d|·|s_d+10| / 20 = 2,959`.
5. **Tustin (T = 10 ms).** `C(z) = (2,988 z − 2,929)/(z − 1)`, o que dá
   `u[k] = u[k−1] + 2,988 e[k] − 2,929 e[k−1]`.
6. **Resultado.**

   | | Sobressinal | ts (2 %) |
   |---|---|---|
   | Contínuo | 6,8 % | 0,80 s |
   | Discreto | 7,8 % | 0,78 s |

   Os dois atendem às especificações.

## Perguntas prováveis na apresentação

- **Por que Tustin?** Ele mapeia o semiplano esquerdo de s para dentro do círculo unitário em z, então preserva a estabilidade. Também aproxima bem a resposta em frequência quando T é pequeno.
- **Por que o discreto tem um pouco mais de sobressinal?** O ZOH equivale a um atraso de cerca de T/2, o que reduz a margem de fase.
- **Por que T = 10 ms?** ωn ≈ 7,7 rad/s, ou seja, cerca de 1,2 Hz. Amostrar a 100 Hz fica muito acima da regra de 20 a 30 amostras por ciclo de banda.
- **Cancelamento polo-zero é seguro?** É, porque o polo cancelado (−2) é estável. Se o modelo real tiver o polo em −2,2, sobra um par polo-zero muito próximo que tem pouco efeito na resposta.
- **Anti-windup:** o Arduino guarda o u já saturado no histórico, o que impede o integrador de acumular erro durante a saturação.
