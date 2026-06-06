<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        Hasta ahora has escrito "scripts": una lista de instrucciones que se ejecutan de arriba a
        abajo. Pero en robótica, el código se vuelve complejo muy rápido.
        <br /><br />
        Para organizar el caos, usamos la **Programación Orientada a Objetos (POO)**. Es la forma de
        empaquetar datos (sensores) y acciones (motores) en una sola estructura lógica.
      </TextBlock>
    </div>

    <!-- 1. LA ANALOGÍA DEL PLANO -->
    <div class="section-group">
      <SectionTitle>1. Clase vs. Objeto</SectionTitle>
      <SplitBlock>
        <template #left>
          <div class="text-h6 text-primary q-mb-sm">La Clase (Class)</div>
          <TextBlock>
            Es el **PLANO** o el diseño.
            <br />
            Define cómo <em>debería</em> ser el robot, pero no existe físicamente. En el plano dice:
            "Todo robot tendrá 2 ruedas y un nombre".
          </TextBlock>
          <div class="q-mt-md code-snippet">
            <code>class Robot: ...</code>
          </div>
        </template>
        <template #right>
          <div class="text-h6 text-secondary q-mb-sm">El Objeto (Object)</div>
          <TextBlock>
            Es el **ROBOT REAL** construido a partir del plano.
            <br />
            Puedes usar el mismo plano para construir 100 robots diferentes (Instancias). Uno se
            puede llamar "Wall-E" y otro "R2D2".
          </TextBlock>
          <div class="q-mt-md code-snippet">
            <code>mi_robot = Robot()</code>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 2. SINTAXIS BÁSICA -->
    <div class="section-group">
      <SectionTitle>2. Tu Primera Clase</SectionTitle>
      <TextBlock>
        Vamos a definir nuestro plano. En Python, usamos la palabra clave <code>class</code>. Por
        convención, los nombres de clases usan <strong>CamelCase</strong> (Mayúscula al inicio).
      </TextBlock>

      <CodeBlock
        title="definicion_simple.py"
        lang="python"
        content="class RobotLimpieza:
    # Atributo de clase (igual para todos)
    tipo = 'Aspiradora'

    # Método simple (una función dentro de la clase)
    def saludar(self):
        print('Hola, estoy listo para limpiar.')

# --- Fuera de la clase (Fábrica) ---

# Crear (Instanciar) el objeto
mi_roomba = RobotLimpieza()

# Usar el objeto
mi_roomba.saludar()
print(mi_roomba.tipo)"
        :copyable="true"
      />
    </div>

    <!-- 3. EL MISTERIOSO 'SELF' -->
    <div class="section-group">
      <SectionTitle>3. ¿Qué rayos es "self"?</SectionTitle>

      <AlertBlock type="warning" title="Concepto Crítico">
        Verás la palabra <code>self</code> en todas partes en ROS 2.
        <br />
        <strong>self</strong> significa "YO MISMO".
      </AlertBlock>

      <TextBlock>
        Imagina que tienes 10 robots idénticos. Si dices "¡Apaga la batería!", ¿cuál de los 10 se
        apaga?
        <br /><br />
        Cuando un robot ejecuta código, usa <code>self</code> para saber que debe apagar
        <strong>SU</strong>
        propia batería y no la del vecino.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="concept-card">
            <div class="text-subtitle1 text-accent">Sin self ❌</div>
            <p>Variable local temporal. Nace y muere dentro de la función.</p>
            <CodeBlock
              lang="python"
              content="def configurar():
    velocidad = 10
    # Al salir de aquí, 'velocidad' desaparece"
            />
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="concept-card">
            <div class="text-subtitle1 text-positive">Con self ✅</div>
            <p>Variable del objeto. Se guarda en la "mochila" del robot para siempre.</p>
            <CodeBlock
              lang="python"
              content="def configurar(self):
    self.velocidad = 10
    # Puedo usar self.velocidad en otras funciones"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- 4. __INIT__: EL NACIMIENTO -->
    <div class="section-group">
      <SectionTitle>4. El Constructor (__init__)</SectionTitle>
      <TextBlock>
        Cuando nace un robot (se crea el objeto), necesitamos configurarlo inicialmente (darle
        nombre, cargar batería). Para eso existe un método especial llamado <code>__init__</code>.
        <br /><br />
        Se ejecuta <strong>automáticamente</strong> una sola vez al crear la instancia.
      </TextBlock>

      <CodeBlock
        title="robot_completo.py"
        lang="python"
        content="class RobotGuerrero:
    def __init__(self, nombre_dado, color_dado):
        # Guardamos los datos recibidos en la memoria del robot (self)
        self.nombre = nombre_dado
        self.color = color_dado
        self.vida = 100  # Valor por defecto
        print(f'¡Ha nacido {self.nombre} de color {self.color}!')

    def recibir_daño(self, cantidad):
        self.vida = self.vida - cantidad
        print(f'{self.nombre} tiene {self.vida} de vida.')

# Creando dos robots distintos con el MISMO plano
r1 = RobotGuerrero('Thor', 'Rojo')
r2 = RobotGuerrero('Hulk', 'Verde')

r1.recibir_daño(20)  # Solo afecta a Thor
r2.recibir_daño(50)  # Solo afecta a Hulk"
        :copyable="true"
      />
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto POO</SectionTitle>
      <TextBlock>
        Crea una clase llamada <code>Dron</code>. 1. En el <code>__init__</code>, recibe una
        <code>altura_maxima</code> e inicia la <code>altura_actual</code> en 0. 2. Crea un método
        <code>volar()</code> que suba la altura actual. 3. Instancia dos drones con diferentes
        alturas máximas.
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

.code-snippet {
  background: var(--bg-surface-hover);
  padding: 8px;
  border-radius: 6px;
  text-align: center;
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.code-snippet code {
  color: #fcd34d;
  font-family: 'Fira Code', monospace;
}

.concept-card {
  background: var(--bg-surface-solid);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1rem;
  height: 100%;
}
</style>
