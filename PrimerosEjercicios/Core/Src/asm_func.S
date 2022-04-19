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
@   r0: vector de entrada
@   r1: longitud del vector
@
@ Valor de retorno:
@   void

.thumb_func
	asm_invertir:

		    push {r4,r5}
		    mov r2, 2
		    mul r1, r2
		    sub r1, 2
		    add r1, r1 , r0
		.itero_invertir:
			ldrh r4, [r0]
			ldrh r5, [r1]
			strh r5, [r0], 2
			strh r4, [r1], -2
			cmp r1, r0
			bcs .itero_invertir
		    pop {r4,r5}
		    bx lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@ void asm_filtroVentana10(uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitudVectorIn);
@
@ Parametros recibidos:
@   r0: vector de entrada de uint16_t
@   r1: vector de salida de uint16_t
@   r2: longitud de los vectores
@   r4: Acumulador
@   r5: Para sumar los valores de vector
@   r6: Contador de ventanas
@
.thumb_func
    asm_filtroVentana10:


		push {r4}        /* hago un push del registro r4*/
		mov r4, 9
		cmp	r2, r4
		bls .asm_fin     /* Finaliza o no la funcion */
		push {r5,r6}
		mov r6, 0        /* pongo a cero los registros r5, r6 */
		mov r4, 0
	.asm_filtroVentana10_loop1:
		mov r5, 10
	.asm_filtroVentana10_loop2:
		ldrh r3, [r0], 2
		add r4, r3
        subs r5, 1	// Cuando llega a cero cambia el flag a 1
		bne .asm_filtroVentana10_loop2
		add r6, 1
		mov r5, 10
		udiv r4,r4,r5 //division
		strh r4,[r1], 2
		add r5, r6
		cmp	r5, r2
		bls .asm_volver_r0
		b .asm_fin
	.asm_volver_r0:
		sub r0, 20
		b .asm_filtroVentana10_loop1
	.asm_fin:
		pop {r4,r6}
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

		push {r4}             // Hago un push de los registro de r4
		mov r4, 0             // inicializo r4 con cero
		ldr r2, [r0], 4
		subs R1, 1
	.asm_max_loop:
		ldr r3, [r0], 4
		cmp r3, r2
		bhi .asm_max_update
		b .asm_max_sigo
	.asm_max_update:          // salvo la posicion y el maximo valor
		add r4, 1
		mov r2, r3
	.asm_max_sigo:
		subs r1, 1            //para saber si ya termino el vector
		bne .asm_max_loop
		mov r0,r4             //guardo en el r0 el indice del maximo valor
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
@   r4:
@   r5:

.thumb_func
	asm_pack32to16:
		push {r4,r5}
	     .pack32to16_loop:
	     ldr r4, [r0], 4       //guardo valor del vector en r4 e incremento el puntero.
	     lsr r5, r4, 16        //corrimiento a la derecha 16 bits.
		 strh r5, [r1], 2      //hago un store de media palabra
		 subs r2, 1            //para saber si ya termino el vector
    	 bne .pack32to16_loop  // loop
		 pop {r4,r5}
		 bx lr //loop

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@ void downsampleM (int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N);
@
@ Parametros recibidos:
@   r0: vector de entrada de int32_t
@   r1: vector de salida de int32_t
@ 	r2: longitud de los vectores uint32_t
@ 	r3: N muestra a quitar uint32_t
@
@
@

.thumb_func
    asm_downsampleM:

		push {r4,r6}                  // Hago un push de los registro
		mov r4, 0                     // Inicializacion de los registros pusheados
		mov r6, 0
	.asm_downsampleM_itero:          // Comparo r4 con el numero de muestra
		cmp r4, r3
		beq .asm_downsampleM_salto    // Si no toca sacar muestra, lo guardo en vectro de salida
		ldr r5, [r0], 4
		str r5,[r1], 4
		add r4, 1
		b .asm_downsampleM_cuenta
	.asm_downsampleM_salto:
		ldr r5, [r0], 4
		mov r4, 0
	.asm_downsampleM_cuenta:
		add r6, 1
		cmp r2, r6
		bne .asm_downsampleM_itero
		pop {r4,r6}
 		bx lr
