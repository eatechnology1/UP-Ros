<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        Ya sabes que las clases son planos. Ahora vamos a centrarnos en el
        <strong>Comportamiento</strong>. Los métodos son las acciones que tu robot puede realizar.
        <br /><br />
        En esta lección, perfeccionaremos el uso de <code>__init__</code> (el nacimiento) y
        crearemos métodos que interactúen entre sí, simulando la lógica real de un nodo de ROS.
      </TextBlock>
    </div>

    <!-- 1. PROFUNDIZANDO EN __INIT__ -->
    <div class="section-group">
      <SectionTitle>1. El "Checklist" de Nacimiento (__init__)</SectionTitle>
      <TextBlock>
        Piensa en el <code>__init__</code> como la lista de comprobación de despegue de un avión.
        Antes de volar, debes encender motores, verificar combustible y probar la radio.
        <br /><br />
        En un programa de robótica, usamos este espacio para establecer el
        <strong>Estado Inicial</strong>.
      </TextBlock>

      <AlertBlock type="info" title="Regla de Oro">
        El <code>__init__</code> nunca debe contener bucles infinitos (como
        <code>while True</code>). Debe ser rápido: configurar y salir. Si te quedas atascado aquí,
        el programa nunca arrancará.
      </AlertBlock>

      <CodeBlock
        title="nave_espacial.py"
        lang="python"
        content="class NaveEspacial:
    def __init__(self, nombre):
        print(f'Inicializando sistemas de {nombre}...')

        # 1. Definir variables de ESTADO (Memoria)
        self.combustible = 100
        self.escudos_activos = False
        self.velocidad = 0

        # 2. Ejecutar configuraciones iniciales
        self.activar_radar() # Llamando a otro método interno

        print('--> Sistemas listos.')

    def activar_radar(self):
        print('   [OK] Radar encendido.')

# Al crear el objeto, todo lo anterior sucede automáticamente
mi_nave = NaveEspacial('Halcón Milenario')"
        :copyable="true"
      />
    </div>

    <!-- 2. MÉTODOS QUE MODIFICAN EL ESTADO -->
    <div class="section-group">
      <SectionTitle>2. Métodos: Acciones y Reacciones</SectionTitle>
      <TextBlock>
        Los métodos no sirven solo para imprimir texto. Su función principal es
        <strong>modificar el estado</strong>
        (las variables `self`) del objeto.
      </TextBlock>

      <SplitBlock>
        <template #left>
          <TextBlock>
            Observa este ejemplo. Tenemos un método <code>acelerar</code>. <br /><br />
            Nota cómo usa <code>self.combustible</code> para decidir si puede acelerar, y luego
            modifica <code>self.velocidad</code>. <br /><br />
            Esto es <strong>Encapsulamiento</strong>: La lógica compleja está oculta dentro del
            método.
          </TextBlock>
        </template>
        <template #right>
          <CodeBlock
            lang="python"
            content="    def acelerar(self):
        if self.combustible > 10:
            self.combustible -= 10
            self.velocidad += 50
            print(f'¡Velocidad actual: {self.velocidad}!')
        else:
            print('¡Alerta! Combustible insuficiente.')"
          />
        </template>
      </SplitBlock>
    </div>

    <!-- 3. PARAMETROS POR DEFECTO -->
    <div class="section-group">
      <SectionTitle>3. Argumentos Opcionales (Flexibilidad)</SectionTitle>
      <TextBlock>
        A veces queremos crear objetos con configuración estándar, pero tener la opción de
        personalizarla si es necesario. Python permite valores por defecto en los métodos.
      </TextBlock>

      <div class="q-my-md">
        <CodeBlock
          title="motor.py"
          lang="python"
          content="class Motor:
    # Si no me dan potencia, asumo que es 100
    def __init__(self, tipo, potencia=100):
        self.tipo = tipo
        self.potencia = potencia

# Caso 1: Solo lo obligatorio
m1 = Motor('Eléctrico')
print(m1.potencia) # Imprime 100

# Caso 2: Personalizado
m2 = Motor('Diesel', potencia=500)
print(m2.potencia) # Imprime 500"
        />
      </div>

      <AlertBlock type="success" title="Aplicación en ROS">
        Esto es utilísimo. Por ejemplo: <code>def __init__(self, nombre_nodo='mi_robot_v1')</code>.
        Si no especificas nombre, usa el default.
      </AlertBlock>
    </div>

    <!-- 4. RETURN VS PRINT -->
    <div class="section-group">
      <SectionTitle>4. Return vs. Print (Error de Novato)</SectionTitle>
      <TextBlock>
        Muchos estudiantes confunden "mostrar un dato" con "devolver un dato".
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="concept-card bad">
            <div class="text-subtitle1 text-white">Solo Print ❌</div>
            <p>El método grita el resultado, pero el programa no puede usarlo para matemáticas.</p>
            <CodeBlock
              lang="python"
              content="def sumar(a, b):
    print(a + b)

resultado = sumar(2, 2)
# ERROR: 'resultado' está vacío (None)"
            />
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="concept-card good">
            <div class="text-subtitle1 text-white">Return ✅</div>
            <p>El método entrega el valor silenciosamente para que el programa siga trabajando.</p>
            <CodeBlock
              lang="python"
              content="def sumar(a, b):
    return a + b

resultado = sumar(2, 2)
print(resultado * 10) # Funciona: 40"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- RETO FINAL -->
    <div class="section-group">
      <SectionTitle>🏆 Reto: El Sensor de Distancia</SectionTitle>
      <TextBlock>
        Crea una clase <code>SensorDistancia</code>: 1. <code>__init__</code>: Recibe
        <code>rango_maximo</code> (default 5.0 metros). 2. Método
        <code>leer_datos(distancia_real)</code>: - Si <code>distancia_real</code> es mayor que
        <code>rango_maximo</code>, devuelve (return) <code>None</code>. - Si es menor, devuelve la
        distancia. - *Pista: Usa un print fuera de la clase para probarlo.*
      </TextBlock>

      <CodeBlock
        title="Solución Esperada"
        lang="python"
        content="class SensorDistancia:
    def __init__(self, rango_maximo=5.0):
        self.max = rango_maximo

    def leer_datos(self, distancia_real):
        if distancia_real > self.max:
            return None
        return distancia_real

sensor = SensorDistancia() # Max default 5.0
print(sensor.leer_datos(3.0)) # Imprime 3.0
print(sensor.leer_datos(8.0)) # Imprime None"
      />
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
  border-radius: 12px;
  padding: 1rem;
  height: 100%;
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.concept-card.bad {
  background: rgba(239, 68, 68, 0.1); /* Rojo suave */
  border-left: 4px solid #ef4444;
}
.concept-card.good {
  background: rgba(34, 197, 94, 0.1); /* Verde suave */
  border-left: 4px solid #22c55e;
}
.concept-card p {
  color: var(--text-secondary);
  font-size: 0.9rem;
  margin: 0.5rem 0;
}
</style>
