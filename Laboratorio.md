# Atividades de Laboratório - Sistemas Embarcados I

1. O programa disponibilizado na pasta **stm32f411-blackpill** pisca o LED conectado ao pino PC13 no kit STM32F411 Blackpill. Faça um *fork* deste repositório e altere o programa para que, ao se pressionar o botão conectado a PA0 o LED fique aceso e, ao soltar este botão, o LED se apague.

R: Para modificar o programa conforme as suas especificações, é necessário alterar o arquivo main.c para incluir a lógica de leitura do botão conectado ao pino PA0 e controlar o LED de acordo com o estado do botão. Aqui está uma possível implementação:

#include "stm32f4xx.h"

void delay_ms(uint32_t ms) {
    // Atraso em milissegundos
    uint32_t delayTicks = ms * (SystemCoreClock / 1000);
    SysTick->LOAD = delayTicks - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;

    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    SysTick->CTRL = 0;
}

int main(void) {
    // Habilita o clock para GPIOA e GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;

    // Configura PA0 como entrada (botão) e PC13 como saída (LED)
    GPIOA->MODER &= ~(GPIO_MODER_MODER0); // PA0 como entrada
    GPIOC->MODER |= GPIO_MODER_MODER13_0; // PC13 como saída

    while (1) {
        // Verifica o estado do botão (PA0)
        if (GPIOA->IDR & GPIO_IDR_IDR_0) {
            // Botão não pressionado, apaga o LED (PC13)
            GPIOC->BSRR |= GPIO_BSRR_BR_13; // Limpa o bit de set do PC13
        } else {
            // Botão pressionado, acende o LED (PC13)
            GPIOC->BSRR |= GPIO_BSRR_BS_13; // Set o bit de set do PC13
        }

        // Atraso para evitar flutuações
        delay_ms(100);
    }
}

Essa implementação verifica continuamente o estado do botão conectado ao pino PA0. Se o botão estiver pressionado, o LED conectado ao pino PC13 é aceso; caso contrário, o LED é apagado.

2. Faça um novo *fork* deste repositório e altere o programa para que, ao se pressionar o botão conectado a PA0 o estado do LED seja trocado, ou seja, caso o LED esteja apagado ao se pressionar o LED uma vez o mesmo deve acender ao pressionar o botão o LED deverá apagar.

R: Para implementar essa funcionalidade, é necessário adicionar uma lógica que alterne o estado do LED sempre que o botão for pressionado. Aqui está uma possível implementação:

#include "stm32f4xx.h"

// Função para configurar o SysTick Timer
void SysTick_Init(void) {
    // Configuração para geração de interrupção a cada 1ms
    SysTick->LOAD = SystemCoreClock / 1000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

// Função para atraso em milissegundos
void delay_ms(uint32_t ms) {
    // Atraso utilizando o SysTick Timer
    uint32_t start = SysTick->VAL;
    uint32_t delay_ticks = ms * (SystemCoreClock / 1000);
    uint32_t end = start - delay_ticks;
    if (end > start) {
        // Sem transbordo
        while (SysTick->VAL > end);
    } else {
        // Com transbordo
        while (SysTick->VAL > end && SysTick->VAL < start);
    }
}

int main(void) {
    // Habilita o clock para GPIOA e GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;

    // Configura PA0 como entrada (botão) e PC13 como saída (LED)
    GPIOA->MODER &= ~(GPIO_MODER_MODER0); // PA0 como entrada
    GPIOC->MODER |= GPIO_MODER_MODER13_0; // PC13 como saída

    // Inicializa o SysTick
    SysTick_Init();

    // Variável para rastrear o estado do LED
    uint8_t led_state = 0;

    // Loop principal
    while (1) {
        // Verifica se o botão está pressionado (PA0)
        if (!(GPIOA->IDR & GPIO_IDR_IDR_0)) {
            // Botão pressionado, alterna o estado do LED
            led_state = !led_state;
            
            // Alterna o estado do LED com base no valor atual da variável led_state
            if (led_state) {
                GPIOC->BSRR |= GPIO_BSRR_BS_13; // Liga o LED (PC13)
            } else {
                GPIOC->BSRR |= GPIO_BSRR_BR_13; // Desliga o LED (PC13)
            }
            
            // Aguarda até que o botão seja liberado para evitar múltiplas trocas de estado
            while (!(GPIOA->IDR & GPIO_IDR_IDR_0));
        }

        // Pequeno atraso para evitar flutuações
        delay_ms(10);
    }
}

Nesta implementação, ao pressionar o botão conectado ao pino PA0, o estado do LED conectado ao pino PC13 é alternado. O estado do LED é controlado pela variável led_state, que é alternada entre 0 e 1 a cada pressionamento do botão.
