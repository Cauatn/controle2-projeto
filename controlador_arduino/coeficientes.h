// Gerado automaticamente por projeto_controlador.py — NÃO edite à mão.
// C(z) obtido por Tustin com T = 0.01 s
// u[k] = sum(B[i]*e[k-i], i=0..NB-1) - sum(A[i]*u[k-i], i=1..NA-1)
#pragma once

#define NB 2
#define NA 2
const float B[NB] = { 2.988165680f, -2.928994083f };
const float A[NA] = { 1.000000000f, -1.000000000f };

const float U_MIN = -12.000f;
const float U_MAX = 12.000f;
const unsigned long T_US = 10000UL;
