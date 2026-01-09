<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        Imagina que estás cocinando. Tienes que vigilar el horno y picar cebolla al mismo tiempo. Si
        te quedas mirando el horno hasta que termine, nunca picarás la cebolla. Tienes que hacer
        ambas cosas "a la vez" (multitasking).
        <br /><br />
        En programación, los programas normales son secuenciales (una cosa a la vez). Si una tarea
        tarda mucho (ej: bajar un archivo), todo el programa se congela. Los **Hilos (Threads)**
        solucionan esto.
      </TextBlock>
    </div>

    <!-- 1. EL PROBLEMA DEL BLOQUEO -->
    <div class="section-group">
      <SectionTitle>1. El Problema: Código Bloqueante</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Mira este código. La función <code>tarea_lenta()</code> simula una operación pesada
            (como procesar una imagen). <br /><br />
            El programa <strong>SE DETIENE</strong> ahí. No puede imprimir "Fin del programa" hasta
            que la tarea lenta termine. En un robot, esto es fatal: si procesar la cámara tarda 1
            segundo, ¡el robot estaría ciego y sordo durante ese segundo!
          </TextBlock>
        </template>
        <template #right>
          <CodeBlock
            title="bloqueo.py"
            lang="python"
            content="import time

def tarea_lenta():
    print('Iniciando tarea pesada...')
    time.sleep(5) # Simula 5 segundos de trabajo
    print('Tarea terminada.')

print('Inicio')
tarea_lenta() # <--- EL CÓDIGO SE CONGELA AQUÍ
print('Fin del programa') # Esto tarda 5 seg en salir"
          />
        </template>
      </SplitBlock>
    </div>

    <!-- 2. SOLUCIÓN CON THREADING -->
    <div class="section-group">
      <SectionTitle>2. La Solución: Multithreading</SectionTitle>
      <TextBlock>
        Con la librería <code>threading</code>, podemos lanzar la tarea lenta en una línea de
        ejecución paralela (un hilo secundario) y dejar que el hilo principal siga trabajando.
      </TextBlock>

      <CodeBlock
        title="multitarea.py"
        lang="python"
        content="import threading
import time

def tarea_lenta():
    print('   [Hilo 2] Iniciando tarea pesada...')
    time.sleep(5)
    print('   [Hilo 2] Tarea terminada.')

print('[Hilo 1] Inicio')

# Preparamos el hilo, pero no lo lanzamos todavía
hilo = threading.Thread(target=tarea_lenta)

# ¡Fuego! Lanzamos el hilo en paralelo
hilo.start()

print('[Hilo 1] Fin del programa (Inmediato)')
# El programa principal termina, pero el Hilo 2 sigue vivo de fondo"
        :copyable="true"
      />

      <AlertBlock type="warning" title="Cuidado con la Competencia">
        Si dos hilos intentan modificar la misma variable al mismo tiempo, ocurren desastres (Race
        Conditions). En ROS 2, esto se gestiona con "Executors" y "Callback Groups", pero el
        concepto base es este.
      </AlertBlock>
    </div>

    <!-- 3. DECORADORES -->
    <div class="section-group">
      <SectionTitle>3. Decoradores (@)</SectionTitle>
      <TextBlock>
        Seguro has visto en tutoriales símbolos raros como <code>@app.route</code> o
        <code>@staticmethod</code>. Son **Decoradores**. <br /><br />
        Un decorador es una función que "envuelve" a otra para agregarle funcionalidad extra sin
        tocar su código original. Es como ponerle una carcasa protectora a tu celular: el celular es
        el mismo, pero ahora es resistente a golpes.
      </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="text-subtitle1 text-primary q-mb-sm">Ejemplo: Medir Tiempo</div>
          <TextBlock>
            Queremos saber cuánto tardan nuestras funciones sin escribir <code>start_time</code> y
            <code>end_time</code> en todas ellas. Creamos un decorador <code>@medir_tiempo</code>.
          </TextBlock>
        </template>
        <template #right>
          <CodeBlock
            lang="python"
            content="import time

# Definimos el decorador
def medir_tiempo(funcion_original):
    def envoltura():
        inicio = time.time()
        funcion_original() # Ejecutamos la original
        fin = time.time()
        print(f'Tardó {fin - inicio} segundos')
    return envoltura

# Usamos el decorador con el símbolo @
@medir_tiempo
def proceso_robot():
    time.sleep(1)
    print('Proceso robot ejecutado')

# Al llamar a la función, ¡ya viene con cronómetro!
proceso_robot()"
          />
        </template>
      </SplitBlock>
    </div>

    <!-- 4. LAMBDA (FUNCIONES ANÓNIMAS) -->
    <div class="section-group">
      <SectionTitle>4. Funciones Lambda (Extra)</SectionTitle>
      <TextBlock>
        En ROS 2 a veces necesitamos funciones pequeñas y desechables para cosas rápidas. Las
        funciones <strong>Lambda</strong> son funciones que no tienen nombre y se escriben en una
        sola línea.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="concept-card">
            <div class="text-subtitle2 text-grey-4">Función Normal</div>
            <CodeBlock
              lang="python"
              content="def cuadrado(x):
    return x * x

print(cuadrado(5))"
            />
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="concept-card highlight">
            <div class="text-subtitle2 text-white">Función Lambda</div>
            <CodeBlock
              lang="python"
              content="cuadrado = lambda x: x * x

print(cuadrado(5))"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- RETO FINAL -->
    <div class="section-group">
      <SectionTitle>🏆 Reto Multitasking</SectionTitle>
      <TextBlock>
        Crea un script con dos funciones: 1. <code>mover_ruedas()</code>: Imprime "Moviendo..." cada
        1 segundo (bucle while). 2. <code>leer_camara()</code>: Imprime "Foto tomada" cada 3
        segundos.
        <br />
        Lanza ambas en hilos separados para que el robot se mueva Y tome fotos simultáneamente.
      </TextBlock>
    </div>
  </div>
</template>

<script setup lang="ts">
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3rem;
}

.concept-card {
  background: rgba(30, 41, 59, 0.5);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1rem;
}
.concept-card.highlight {
  border-color: #f59e0b; /* Amber */
  background: rgba(245, 158, 11, 0.1);
}
</style>
