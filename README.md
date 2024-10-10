# Trea_3
Embedded Peripherals - Exercise
Estos son los requerimientos del sistema:

No funcionales:
1. Tener 3 botones: Giro Izquierda, Giro Derecha, Estacionar.
2. Tener 3 luces(LEDs): Heartbeat del sistema, Luz Izquierda, Luz Derecha.
3. Tener un puerto de depuracion con el PC: USART2

Funcionales:
4. Heartbeat con frecuencia de 1Hz para indicar que el sistema esta funcionando.
5. Si un botón de giro se presiona 1 vez: la luz del lado correspondiente parpadea 3 veces.
6. Si un botón de giro se presiona 2 veces en menos de 300ms: la luz del lado correspondiente parpadea indefinidamente.
7. Si un botón de giro se presiona y la luz del otro lado esta activa: se desactiva la luz.
8. Si se presiona el boton para la señal de estacionamiento: funciona igual que un carro real, ejemplo: Tesla.
9. La frecuencia de parpadeo de las luces debe ser aceptable según "El Reglamento General de Circulación".
10. Se deben poder identificar los eventos principales del sistema en una consola serial de PC.


Criterio de evaluación:
Funcionalidad: 60% (valor equivalente para cada requerimiento)
Repositorio: 20% (commits entendibles que describan los cambios realizados)
Documentación: 20% (en código y readme del repositorio)
