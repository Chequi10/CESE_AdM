.syntax unified
.thumb
.text

@ Simbolos exportados
.global asm_svc
.global asm_sum
.global asm_zeros
.global asm_productoEscalar32
.global asm_productoEscalar16
.global asm_productoEscalar12
.global asm_invertir
.global asm_max
.global asm_pack32to16
.global asm_filtroVentana10
.global asm_downsampleM

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void asm_svc (void)
@
.thumb_func
    asm_svc:
        svc 0
        bx lr


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   uint32_t asmSum (uint32_t firstOperand, uint32_t secondOperand)
@
@ Parametros recibidos:
@   r0: firstOperand
@   r1: secondOperand
@
@ Valor de retorno:
@   r0: resultado de la suma de firstOperand y secondOperand
@
.thumb_func
    asm_sum:
        add r0, r1  @ r0 = r0 + r1
        bx lr       @ vuelve adonde fue llamada (especificamente, si "pc"
                    @ es la direccion, en el codigo del programa, de la
                    @ instruccion "bl asm_sum" usada para saltar a
                    @ esta funcion, entonces "lr" contiene ese "pc" + 4).
                    @ el valor de retorno debe quedar en el registro r0.
                    @ En este caso, r0 es el resultado de r0 + r1.

        @ Otras alternativas para el retorno
        @ mov pc,lr
        @ bx lr
        @ push {lr} ... pop {pc}



@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void asm_zeros (uint32_t * vector, uint32_t longitud);
@
@ Parametros recibidos:
@   r0: puntero al vector
@   r1: longitud del vector
@
@
.thumb_func
    asm_zeros:
        mov r2, 0
    .asm_zeros_for:
        str r2, [r0], 4 //*vecrtor++ = r2 // *vector = r2
        subs r1, 1      // --longitud
        bne .asm_zeros_for
        bx lr

        @ Otras alternativas para el retorno
        @ mov pc,lr
        @ bx lr
        @ push {lr} ... pop {pc}


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@  void asm_productoEscalar32 (uint32_t * vectorIn, uint32_t * vectorOut, uint32_t longitud, uint32_t escalar);
@
@ Parametros recibidos:
@   r0: puntero al vector entrada
@   r1: puntero al vector salida
@   r2: longitud de los vectores
@   r3: escalar

.thumb_func
    asm_productoEscalar32:
        push {r5,r6}
    .asm_productoEscalar32_bucle:
        ldr r6, [r0], 4
        mul r5, r6,r3
        str r5, [r1], 4
        subs r2, 1
        bne .asm_productoEscalar32_bucle
        pop {r5,r6}
        bx lr

        @ Otras alternativas para el retorno
        @ mov pc,lr
        @ bx lr
        @ push {lr} ... pop {pc}




@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@  void asm_productoEscalar16 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);
@
@ Parametros recibidos:
@   r0: puntero al vector entrada
@   r1: puntero al vector salida
@   r2: longitud de los vectores
@   r3: escalar

.thumb_func
    asm_productoEscalar16:
        push {r5,r6}
    .asm_productoEscalar16_bucle:
        ldrh r6, [r0], 2
        mul r5, r6, r3
        strh r5, [r1], 2
        subs r2, 1
        bne .asm_productoEscalar16_bucle
        pop {r5,r6}
        bx lr

        @ Otras alternativas para el retorno
        @ mov pc,lr
        @ bx lr
        @ push {lr} ... pop {pc}



@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@  void asm_productoEscalar12 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);

@
@ Parametros recibidos:
@   r0: puntero al vector entrada
@   r1: puntero al vector salida
@   r2: longitud de los vectores
@   r3: escalar

.thumb_func
    asm_productoEscalar12:
        push {r4,r6}
        mov r6, 4095
    .asm_productoEscalar12_bucle:
    	ldrh r4, [r0], 2
      	mul r5, r4, r3
    	and r5, r5, r6
    	strh r5, [r1], 2
    	subs r2, 1
    	bne .asm_productoEscalar12_bucle
      	pop {r4,r6}
      	bx lr


        @ Otras alternativas para el retorno
        @ mov pc,lr
        @ bx lr
        @ push {lr} ... pop {pc}



@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@ void asm_invertir (uint16_t * vector, uint32_t longitud);
@
@ Parametros recibidos:
@   r0: Primer operando - vector de entrada de int16_t
@   r1: Segundo operando - longitud del vector
@
@ Valor de retorno:
@   void

.thumb_func
    asm_invertir:

		/* Hago un push de los registro de r4 a r5 */
		push {r4,r5}
		/* Muevo al registro r4 el valor del registro r1=longitud del vector  */
		mov r4, r1
		subs r4, 1 // voy restando hasta llegar a recorrer el vector
		mov r3,2
		mul r4, r3  // Al ser uint16_t multiplico x 2
		/* Apunto el R2 al final del vector */
		mov r2, r0
		add r2, r4
		/* Le resto 2 a la cantidad de combinaciones a iterar
		 * Para 3 elementos: 1 iteracion
		 * Para 4 elementos: 2 iteraciones
		 * Para N elementos: N - 2 iteraciones
		 */
		subs r1, 8

	.asm_invertir_loop:
		ldrh r4, [r2]	// Copio el valor del extremo derecho del vector
		ldrh r5, [r0]	// Copio el valor del extremo izquierdo del vector
		/* Intercambio valores entre las puntas */
		strh r5, [r2],-2
		strh r4, [r0], 2

		subs r1, 1	// Cuando llega a cero cambia el flag a 1
		bne .asm_invertir_loop

		pop {r4,r5}

 		bx lr






@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":

@ int32_t asm_max(int32_t * vectorIn, uint32_t longitud);
@
@ Parametros recibidos:
@   r0: vector de uint32_t
@   r1: vector de uint32_t
@   r2: valor maximo
@   r3: para recorrer el vector
@   r4: posicion donde esta el maximo valor

.thumb_func
    asm_max:

		/* Hago un push de los registro de r4 */
		push {r4}
		mov r4, 0
		ldr r2, [r0], 4
		subs R1, 1
	.asm_max_loop:
		ldr r3, [r0], 4
		cmp r3, r2
		bhi .asm_max_update
		b .asm_max_sigo

	/* salvo la posicion y el maximo valor */
	.asm_max_update:
		add r4, 1
		mov r2, r3

	/* Hago la diferencia para poder saber si ya recorrio el vectro */
	.asm_max_sigo:
		subs r1, 1
		bne .asm_max_loop

	/* Copio la posicion del maximo al registro de salida */
		mov r0,r4

		pop {r4}

        bx lr



@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@
@ void pack32to16 (int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud);
@
@ Parametros recibidos:
@   r0: vector de entrada de int32_t
@   r1: vector de salida de int16_t
@   r2: longitud del vector
@

.thumb_func
    asm_pack32to16:

	ldr r3, [r0], 4	//Salvo e incremento el puntero.
	lsr r3, R3, 16	//Hago corrimiento a la derecha 16 bits
	str r3, [r1], 2
	bx lr
