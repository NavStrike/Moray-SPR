# Hechos:
* Corrección temporal del establecimiento del 0 relativo.
* Integrar el postprocesamiento.
* Arreglar los lanzadores (.sh):
  - Eliminar linea:
    QT_QPA_GENERIC_PLUGINS=evdevtouch:/dev/input/event5

# Urgente:
* Pasar el postprocesamiento a python.
* Adicionar funciones de visualización y crear barra para su gestión.
  - regresar imagen a posición defecto
  - zomm +
  - zoom -
  - mover
* Formatear a mseg, seg, min y horas el tiempo mostrado barra general.
* Permitir crear marcas en la gráfica durante la ejecución.
* Marcar pestaña activa con un rectangulo al cotado izquierdo.
* Botón para actualizar la aplicación (update).
* Mover el botón de borrar 1 ciclo.

# Lista de mejoras:
* Mejorar la gestion de variables: uso de un sistema de carpetas.
* Volcado de la logica en python (usar la GUI con QML solo para mostrar los datos).
* Crear un archivo Json para almacenar datos.
* Crear un nuevo hilo para adquirir datos.