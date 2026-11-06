/*
 * Controlador digital C(z) (projetado por lugar das raízes + Tustin)
 * rodando no Arduino em esquema hardware-in-the-loop.
 *
 * A planta é simulada no PC (hil_planta.py). A cada período T:
 *   PC -> Arduino :  "r y\n"        (referência e saída medida da planta)
 *   Arduino -> PC :  "u dt_us\n"    (sinal de controle e intervalo medido entre amostras)
 *   PC -> Arduino :  "R\n"          (zera o estado do controlador; responde "OK")
 *
 * Os coeficientes vêm de coeficientes.h, gerado por projeto_controlador.py.
 * Equação de diferenças:
 *   u[k] = B0 e[k] + B1 e[k-1] + ... - A1 u[k-1] - ...
 */
#include "coeficientes.h"

float e_hist[NB];   // e[k], e[k-1], ...
float u_hist[NA];   // u[k], u[k-1], ...
unsigned long t_anterior = 0;

void zerar() {
  for (int i = 0; i < NB; i++) e_hist[i] = 0.0f;
  for (int i = 0; i < NA; i++) u_hist[i] = 0.0f;
  t_anterior = micros();
}

float controlador(float r, float y) {
  // desloca históricos
  for (int i = NB - 1; i > 0; i--) e_hist[i] = e_hist[i - 1];
  for (int i = NA - 1; i > 0; i--) u_hist[i] = u_hist[i - 1];

  e_hist[0] = r - y;

  float u = 0.0f;
  for (int i = 0; i < NB; i++) u += B[i] * e_hist[i];
  for (int i = 1; i < NA; i++) u -= A[i] * u_hist[i];

  // saturação; guardar o valor JÁ saturado funciona como anti-windup do PI
  if (u > U_MAX) u = U_MAX;
  if (u < U_MIN) u = U_MIN;

  u_hist[0] = u;
  return u;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  zerar();
  Serial.println("PRONTO");
}

void loop() {
  if (!Serial.available()) return;

  String linha = Serial.readStringUntil('\n');
  linha.trim();
  if (linha.length() == 0) return;

  if (linha[0] == 'R') {
    zerar();
    Serial.println("OK");
    return;
  }

  int sep = linha.indexOf(' ');
  if (sep < 0) return;
  float r = linha.substring(0, sep).toFloat();
  float y = linha.substring(sep + 1).toFloat();

  unsigned long agora = micros();
  unsigned long dt = agora - t_anterior;
  t_anterior = agora;

  float u = controlador(r, y);

  Serial.print(u, 6);
  Serial.print(' ');
  Serial.println(dt);
}
