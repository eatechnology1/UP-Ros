<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        Imagina que eres un ingeniero genético. Tienes el ADN de un "Lobo". Quieres crear un
        "Perro". No empiezas desde cero (célula por célula). Tomas el ADN del Lobo (que ya sabe
        comer, dormir y correr) y solo modificas la parte de "ser salvaje" por "ser amigable".
        <br /><br />
        En programación, esto es <strong>Herencia</strong>. Creamos clases nuevas basadas en otras
        que ya existen.
      </TextBlock>
    </div>

    <!-- 1. EL CONCEPTO VISUAL -->
    <div class="section-group">
      <SectionTitle>1. Padre e Hijo (Superclase y Subclase)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            <strong>Clase Padre (Base):</strong> Tiene las funciones generales. (Ej: Vehículo).
            <br /><br />
            <strong>Clase Hija (Derivada):</strong> Hereda TODO lo del padre y agrega cosas
            específicas. (Ej: Coche, Barco). <br /><br />
            La sintaxis es: <code>class Hijo(Padre):</code>
          </TextBlock>
        </template>
        <template #right>
          <div class="bg-dark q-pa-md rounded-borders text-center">
            <div class="text-h6 text-white q-mb-md">Jerarquía</div>
            <!-- Diagrama simple con HTML/CSS -->
            <div class="tree-node parent">Vehículo (Motor, Ruedas)</div>
            <div class="tree-connector">|</div>
            <div class="tree-row">
              <div class="tree-node child">Coche (+Aire Acond)</div>
              <div class="tree-node child">Moto (+Manillar)</div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 2. SINTAXIS EN CÓDIGO -->
    <div class="section-group">
      <SectionTitle>2. Herencia en Acción</SectionTitle>
      <TextBlock>
        Vamos a crear una clase base <code>Robot</code> y una subclase <code>RobotVolador</code>.
      </TextBlock>

      <CodeBlock
        title="herencia_robots.py"
        lang="python"
        content="# --- CLASE PADRE ---
class Robot:
    def __init__(self, nombre):
        self.nombre = nombre
        self.energia = 100

    def saludar(self):
        print(f'Soy {self.nombre}, un robot genérico.')

# --- CLASE HIJA ---
class RobotVolador(Robot): # <--- ¡AQUÍ ESTÁ LA MAGIA!
    def volar(self):
        print(f'{self.nombre} está volando por los aires.')

# --- PRUEBA ---
dron = RobotVolador('SkyBot')

# El hijo tiene sus propios métodos
dron.volar()

# ¡Y TAMBIÉN tiene los del padre!
dron.saludar()
print(dron.energia)"
        :copyable="true"
      />
    </div>

    <!-- 3. SUPER(): LLAMANDO AL PADRE -->
    <div class="section-group">
      <SectionTitle>3. La función super()</SectionTitle>

      <AlertBlock type="danger" title="⚠️ El Problema de Sobrescribir">
        Si la clase hija tiene su propio <code>__init__</code>, <strong>borra</strong> el
        <code>__init__</code> del padre. El padre nunca se inicializa, y el robot nace roto.
      </AlertBlock>

      <TextBlock>
        Para arreglar esto, usamos <code>super().__init__()</code>. Es como decir: "Antes de hacer
        mis cosas de hijo, asegúrate de hacer lo que hacía papá". <br /><br />
        <strong>Esto es obligatorio en ROS 2.</strong>
      </TextBlock>

      <CodeBlock
        title="super_init.py"
        lang="python"
        content="class RobotAcuatico(Robot):
    def __init__(self, nombre, profundidad_max):
        # 1. Primero inicializamos la parte de 'Robot' genérico
        super().__init__(nombre)

        # 2. Ahora añadimos lo específico
        self.profundidad = profundidad_max

submarino = RobotAcuatico('Nautilus', 500)
print(submarino.nombre)    # Viene del Padre (gracias a super)
print(submarino.profundidad) # Viene del Hijo"
      />
    </div>

    <!-- 4. POR QUÉ ES VITAL EN ROS 2 -->
    <div class="section-group">
      <SectionTitle>4. La Conexión con ROS 2</SectionTitle>
      <TextBlock>
        Aquí es donde todo cobra sentido. En ROS 2, no escribimos el código de comunicación desde
        cero. Usamos una clase maestra llamada <code>Node</code>.
      </TextBlock>

      <div class="q-my-md">
        <CodeBlock
          title="Estructura Real de un Nodo ROS 2"
          lang="python"
          content="from rclpy.node import Node

class MiNodoCamara(Node): # Heredamos de Node
    def __init__(self):
        # Llamamos al constructor de Node y le damos nombre al nodo
        super().__init__('nodo_camara')

        # Ahora nuestro objeto 'MiNodoCamara' tiene superpoderes:
        # self.create_publisher()
        # self.create_timer()
        # self.get_logger().info('¡Hola ROS!')"
        />
      </div>

      <AlertBlock type="success" title="Revelación">
        Al hacer <code>super().__init__('nombre')</code>, estás conectando tu script al sistema
        nervioso de ROS. Sin esa línea, tu clase es solo un script de Python aislado.
      </AlertBlock>
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto de Herencia</SectionTitle>
      <TextBlock>
        1. Crea una clase padre <code>Vehiculo</code> con un atributo <code>ruedas</code>. 2. Crea
        una clase hija <code>Moto</code> que herede de Vehículo. 3. En el <code>__init__</code> de
        Moto, usa <code>super()</code> para configurar 2 ruedas, y añade un atributo nuevo
        <code>casco_puesto = True</code>.
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

/* Estilos para el diagrama de árbol */
.tree-node {
  background: #334155;
  padding: 8px 16px;
  border-radius: 6px;
  display: inline-block;
  border: 1px solid #475569;
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
}
.tree-node.parent {
  background: #1e293b;
  border-color: #38bdf8;
  color: #38bdf8;
}
.tree-node.child {
  background: #1e293b;
  border-color: #4ade80;
  color: #4ade80;
}
.tree-connector {
  color: #64748b;
  font-weight: bold;
  margin: 4px 0;
}
.tree-row {
  display: flex;
  justify-content: center;
  gap: 20px;
}
</style>
