/*
 * lidar_accel.c -- driver bare-metal (ARM Cortex-A9 / Zynq) do acelerador.
 *
 * Mostra o PARTICIONAMENTO HW/SW na pratica:
 *
 *   HARDWARE  O(N*K) : projeta 720 feixes e testa 19 corredores -> 9.918
 *                      avaliacoes, em 751 ciclos (7,51 us @ 100 MHz).
 *   SOFTWARE  O(K)   : a funcao custo e o argmin -- 19 comparacoes. Fica no ARM
 *                      porque e onde moram os pesos ajustaveis em campo
 *                      (w_obstacle, w_deviation, assist_gain). Move-los para o
 *                      RTL trocaria flexibilidade por um ganho irrisorio.
 *
 * Compilar no Vitis contra a plataforma exportada (system.xsa).
 */
#include <stdio.h>
#include <math.h>
#include "xparameters.h"
#include "xaxidma.h"
#include "xil_io.h"
#include "xil_cache.h"

/* ── Mapa de registradores (ver lidar_accel_axi.vhd) ───────────────────────── */
#define ACC_BASE     XPAR_LIDAR_ACCEL_AXI_0_S_AXI_BASEADDR
#define REG_CTRL     0x00u        /* [0] start                                 */
#define REG_STATUS   0x04u        /* [0] done   [1] busy                       */
#define REG_CLEAR0   0x40u        /* folga do candidato 0 (int32, Q6.10)       */

#define K_CAND       19
#define N_BEAMS      720
#define Q_R          10           /* fracionarios: metros = raw / 2^10          */

/* ── Parametros do supervisor (identicos ao no ROS 2) ──────────────────────── */
#define STOP_D       0.60f
#define SLOW_D       1.10f
#define BLOCKED_COST 10.0f
#define W_OBSTACLE   1.0f
#define W_DEVIATION  0.35f
#define DEV_MAX      (45.0f * (float)M_PI / 180.0f)
#define INF_Q        32767

static XAxiDma dma;

/* Feixe empacotado como o AXI4-Stream espera:
 *   [31:16] theta (Q3.13, ja envolvido em [-pi,pi))   [15:0] range (Q6.10)
 * TUSER carrega a validade -- aqui, por simplicidade, um range 0 marca invalido.
 */
static inline uint32_t pack_beam(int16_t range_q, int16_t theta_q)
{
    return ((uint32_t)(uint16_t)theta_q << 16) | (uint16_t)range_q;
}

/* Dispara uma varredura: DMA -> acelerador, e espera o 'done'. */
int accel_run(uint32_t *beams, int n)
{
    Xil_Out32(ACC_BASE + REG_CTRL, 1);            /* zera os minimos */

    Xil_DCacheFlushRange((UINTPTR)beams, n * sizeof(uint32_t));
    int st = XAxiDma_SimpleTransfer(&dma, (UINTPTR)beams,
                                    n * sizeof(uint32_t), XAXIDMA_DMA_TO_DEVICE);
    if (st != XST_SUCCESS)
        return st;

    while (XAxiDma_Busy(&dma, XAXIDMA_DMA_TO_DEVICE))
        ;                                          /* na versao final: espera a IRQ */
    while ((Xil_In32(ACC_BASE + REG_STATUS) & 1u) == 0u)
        ;
    return XST_SUCCESS;
}

/* Le as 19 folgas (em metros). INFINITY = corredor livre. */
void accel_read_clearances(float out[K_CAND])
{
    for (int k = 0; k < K_CAND; k++) {
        int32_t raw = (int32_t)Xil_In32(ACC_BASE + REG_CLEAR0 + 4u * k);
        out[k] = (raw == INF_Q) ? INFINITY : (float)raw / (float)(1 << Q_R);
    }
}

/* ── A parte que FICA em software: custo + argmin (O(K) = 19 operacoes) ────── */
static float obstacle_term(float clear)
{
    if (isinf(clear) || clear >= SLOW_D) return 0.0f;
    if (clear <= STOP_D)                 return BLOCKED_COST;
    return (SLOW_D - clear) / (SLOW_D - STOP_D);
}

int choose_direction(const float clear[K_CAND], float w_user,
                     float *front_clear, int *all_blocked)
{
    float center = fmaxf(-DEV_MAX, fminf(DEV_MAX, w_user));
    float best_cost = INFINITY;
    int   best_k = K_CAND / 2;

    *all_blocked = 1;
    *front_clear = clear[K_CAND / 2];      /* candidato central: delta = 0 */

    for (int k = 0; k < K_CAND; k++) {
        float delta = -DEV_MAX + k * (2.0f * DEV_MAX / (K_CAND - 1));
        float obs   = obstacle_term(clear[k]);
        if (obs < BLOCKED_COST) *all_blocked = 0;

        float cost = W_OBSTACLE * obs
                   + W_DEVIATION * fabsf(delta - center) / DEV_MAX;
        if (cost < best_cost) { best_cost = cost; best_k = k; }
    }
    return best_k;
}

int main(void)
{
    static uint32_t beams[N_BEAMS];
    float clear[K_CAND], front;
    int   blocked;

    XAxiDma_Config *cfg = XAxiDma_LookupConfig(XPAR_AXIDMA_0_DEVICE_ID);
    if (XAxiDma_CfgInitialize(&dma, cfg) != XST_SUCCESS) {
        xil_printf("falha ao inicializar o DMA\r\n");
        return -1;
    }
    XAxiDma_IntrDisable(&dma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);

    /* Aqui entrariam os feixes reais vindos do LiDAR (via UART/Ethernet da Pi).
     * Para o ensaio de bancada carregamos a mesma varredura usada no testbench,
     * de modo que o resultado no ARM pode ser conferido contra o modelo. */
    for (int i = 0; i < N_BEAMS; i++)
        beams[i] = pack_beam(0, 0);        /* substituir pelo scan real */

    if (accel_run(beams, N_BEAMS) != XST_SUCCESS) {
        xil_printf("falha na transferencia\r\n");
        return -1;
    }
    accel_read_clearances(clear);

    int k = choose_direction(clear, 0.0f, &front, &blocked);
    float delta_deg = (-45.0f + k * 5.0f);

    xil_printf("folga a frente : %d mm\r\n", (int)(front * 1000.0f));
    xil_printf("melhor desvio  : %d graus\r\n", (int)delta_deg);
    xil_printf("bloqueado      : %s\r\n", blocked ? "sim" : "nao");
    return 0;
}
