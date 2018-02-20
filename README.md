# mbed-EV3UARTSensor
Librería para la adquisición de datos de sensores de color EV3 de LEGO. Compatible con Mbed.

## Dependencias
<https://github.com/fpinna13/SysTimer>

**Advertencia**:
> No utilizar funciones de retardo(wait,wait_ms,wait_us) que provee Mbed, ya que estas generan conflicto con los temporizadores periodicos(Ticker). 
En su lugar, usar la función delay_ms proveída por esta librería.

## Instalación
### Mbed Compiler
1. Import.
2. Import from URL.
3. Agregar el URL del repositorio.

### Platformio
```
platformio lib install https://github.com/fpinna13/SysTimer.git
platformio lib install https://github.com/fpinna13/mbed-EV3UARTSensor.git
```
**Nota**:
Requiere tener git instalado.

## Diagrama de conexión
Los colores de las conexiones se corresponden con los colores de los cables de LEGO.
<img src="https://user-images.githubusercontent.com/19673895/36406509-303de414-15d6-11e8-8e5e-6ff5637c6e45.png" width="700" height="500" />

**Advertencia**:
> La tensión de bus del sensor de color EV3 es de 5v.
