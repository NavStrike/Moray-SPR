# Hechos:
* Corrección temporal del establecimiento del 0 relativo.
* Arreglar los lanzadores (.sh):
  - Eliminar linea:
    QT_QPA_GENERIC_PLUGINS=evdevtouch:/dev/input/event5

# Haciendo
* Integrar el postprocesamiento -> Código Python
* Eliminación del parámetro n_cycle 
* Cambio del tipo de archivo de guardado: csv, HDF5 y parquet
* Instalación de la librería para leer archivo .h5: pip install h5py
* Adición de un módulo de hora al prototipo (modificación física)
+ Dejar de rastrear archivos inútiles: modificación del gitignore

# Urgente:
* Adicionar funciones de visualización y crear barra para su gestión.
  - regresar imagen a posición defecto
  - zomm +
  - zoom -
  - mover
  x retroceder
  x cambiar gráfica
* Formatear a mseg, seg, min y horas el tiempo mostrado barra general.
* Permitir crear marcas en la gráfica durante la ejecución.
* Marcar pestaña activa con un rectangulo al costado izquierdo.
* Botón para actualizar la aplicación (update).
* Mover el botón de borrar 1 ciclo.
* Creación de margenes visualización de la gráfica
* Que los valores del reseteo cambien los valores graficamente

# Lista de mejoras:
* Mejorar la gestion de variables: uso de un sistema de carpetas.
* Volcado de la logica en python (usar la GUI con QML solo para mostrar los datos).
* Crear un archivo Json para almacenar datos.
* Crear un nuevo hilo para adquirir datos.
* Corrección permanente del establecimiento del 0 relativo.
* Colocar un reloj.
* Crear teclado flotante.
* Mejora el segmentado de la gráfica:
  // En horizontal
  const step=1;
  const nTicks=Math.floor(xmax/step)-Math.ceil(xmax/step);
  if (20<nTicks){step=0.5}
  const firstTick=Math.ceil(xmin/step)*step;
  const lastTick=Math.floor(xmax/step)*step;
  ctx.beginPath();

  for (let v=firstTick; v<=lastTick; v+=step){
    const x=xMap(v);
    ctx.moveTo(x,mTop);
    ctx.lineTo(x,mTop+ZoneH);
  }
  ctx.strokeStyle="#253041";
  ctx.stroke();