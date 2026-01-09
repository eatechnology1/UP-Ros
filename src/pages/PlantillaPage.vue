<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION (Estándar) -->
    <section class="intro-hero self-stretch q-mb-xl">
      <div class="hero-content text-center">
        <div class="text-overline text-accent text-weight-bold">Master Template</div>
        <h1 class="hero-title">Biblioteca de <span class="text-primary">Componentes</span></h1>
        <p class="text-grey-4 text-subtitle1 q-mt-sm">
          Referencia técnica completa de todos los bloques disponibles en UP-Ros Academy.
        </p>
      </div>
    </section>

    <!-- CONTENEDOR PRINCIPAL -->
    <div class="content-wrapper self-stretch">
      <!-- A. TIPOGRAFÍA Y TEXTO -->
      <SectionTitle>A. Bloques de Texto</SectionTitle>
      <TextBlock>
        El componente <code>&lt;TextBlock&gt;</code> es el ladrillo básico. Maneja el espaciado y la
        legibilidad automáticamente. Soporta <strong>negritas</strong>, <em>cursivas</em> y
        <span class="text-secondary">colores de Quasar</span>.
      </TextBlock>

      <!-- B. ALERTAS / NOTIFICACIONES -->
      <SectionTitle>B. Alertas Semánticas</SectionTitle>
      <div class="q-gutter-y-md">
        <AlertBlock type="info" title="Información (Info)">
          Ideal para notas al margen, consejos o contexto adicional no crítico.
        </AlertBlock>
        <AlertBlock type="success" title="Éxito (Success)">
          Para confirmar pasos completados, resultados esperados o buenas prácticas.
        </AlertBlock>
        <AlertBlock type="warning" title="Advertencia (Warning)">
          Para llamar la atención sobre requisitos previos o configuraciones delicadas.
        </AlertBlock>
        <AlertBlock type="danger" title="Peligro (Danger)">
          Para errores fatales, comandos destructivos o zonas de depuración.
        </AlertBlock>
      </div>

      <!-- C. CÓDIGO Y TERMINAL -->
      <SectionTitle>C. Bloques de Código</SectionTitle>

      <!-- C1. Python con Copiado -->
      <div class="code-label python">🐍 Python (Ejemplo con Label)</div>
      <CodeBlock
        title="node_example.py"
        lang="python"
        content="import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('Hello World')"
        :copyable="true"
      />

      <div class="q-mt-lg"></div>

      <!-- C2. Terminal sin Copiado -->
      <CodeBlock
        title="Terminal (Solo Lectura)"
        lang="bash"
        content="$ sudo apt install ros-jazzy-desktop
[sudo] password for user:
Reading package lists... Done"
        :copyable="false"
      />

      <!-- D. LISTAS Y PASOS -->
      <SectionTitle>D. Listas de Pasos</SectionTitle>
      <StepsBlock
        :steps="[
          'Primer paso: Importar librerías.',
          'Segundo paso: Definir la clase del nodo.',
          'Tercer paso: Crear el bloque main().',
          'Cuarto paso: Ejecutar colcon build.',
        ]"
      />

      <!-- E. ESTRUCTURA DIVIDIDA Y MULTIMEDIA -->
      <SectionTitle>E. Split Layout & Multimedia</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            <strong>SplitBlock</strong> es vital para tutoriales. A la izquierda explicamos la
            teoría y a la derecha mostramos la evidencia visual. <br /><br />
            Abajo tenemos un <code>&lt;ImageBlock&gt;</code> con capacidad de zoom.
          </TextBlock>
        </template>
        <template #right>
          <ImageBlock
            src="src/assets/images/jazzy_robot.png"
            caption="Robot Jazzy (Clic para Zoom)"
            :zoomable="true"
          />
        </template>
      </SplitBlock>

      <!-- F. TARJETAS PERSONALIZADAS (Ej: Glosario/Proyectos) -->
      <SectionTitle>F. Tarjetas Personalizadas</SectionTitle>
      <div class="row q-col-gutter-md">
        <!-- Tarjeta Estilo Glosario -->
        <div class="col-12 col-md-6">
          <div class="custom-card border-purple">
            <div class="card-header">
              <div class="text-h6 text-white">Nodo (Core)</div>
              <q-badge color="purple">Core</q-badge>
            </div>
            <p class="text-grey-4">Unidad mínima de procesamiento en ROS.</p>
          </div>
        </div>
        <!-- Tarjeta Estilo Proyecto -->
        <div class="col-12 col-md-6">
          <div class="custom-card border-orange">
            <div class="card-header">
              <div class="text-h6 text-white">Servicio (Comm)</div>
              <q-badge color="orange">Comm</q-badge>
            </div>
            <p class="text-grey-4">Comunicación síncrona Cliente/Servidor.</p>
          </div>
        </div>
      </div>

      <!-- G. BUSCADOR INTERACTIVO (Del Glosario) -->
      <SectionTitle>G. Componente de Búsqueda</SectionTitle>
      <div class="bg-dark q-pa-lg rounded-borders border-white-10">
        <TextBlock>
          Este es el motor de búsqueda aislado. Escribe "Nodo" o "ROS" para probar el filtrado en
          tiempo real.
        </TextBlock>

        <!-- INPUT BUSCADOR -->
        <div class="search-container q-my-md">
          <q-input
            v-model="search"
            dark
            outlined
            rounded
            placeholder="Probar buscador..."
            class="search-input"
            bg-color="blue-grey-10"
          >
            <template v-slot:prepend><q-icon name="search" color="primary" /></template>
            <template v-slot:append>
              <q-icon
                v-if="search"
                name="close"
                color="grey"
                class="cursor-pointer"
                @click="search = ''"
              />
            </template>
          </q-input>
        </div>

        <!-- RESULTADOS DEMO -->
        <div class="glossary-grid q-mt-md" v-if="search">
          <div v-for="term in filteredDemoTerms" :key="term" class="custom-card border-green">
            <div class="text-bold text-white">{{ term }}</div>
            <div class="text-caption text-grey-5">Resultado de ejemplo</div>
          </div>
        </div>
        <div v-else class="text-center text-grey-6 text-caption">
          (Los resultados aparecerán aquí al escribir)
        </div>
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue';

// IMPORTAR TODOS LOS COMPONENTES (Sin PageFooterNav)
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import StepsBlock from 'components/content/StepsBlock.vue';
import ImageBlock from 'components/content/ImageBlock.vue';
import SplitBlock from 'components/content/SplitBlock.vue';

// LOGICA DEL BUSCADOR DEMO
const search = ref('');
const demoTerms = ['Nodo', 'Tópico', 'Servicio', 'ROS 2', 'Gazebo', 'Python', 'C++'];

const filteredDemoTerms = computed(() => {
  if (!search.value) return [];
  return demoTerms.filter((t) => t.toLowerCase().includes(search.value.toLowerCase()));
});
</script>

<style scoped>
.content-wrapper {
  width: 100%;
  max-width: 1000px;
  margin: 0 auto;
}

/* HERO */
.intro-hero {
  padding: 3rem 1rem;
  background: radial-gradient(circle at center, rgba(99, 102, 241, 0.15), transparent 60%);
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 16px;
}

.hero-title {
  font-size: 3rem;
  font-weight: 800;
  color: #f8fafc;
  line-height: 1.1;
  margin: 10px 0;
}

/* ETIQUETAS DE CÓDIGO */
.code-label {
  display: inline-block;
  padding: 4px 12px;
  border-radius: 6px 6px 0 0;
  font-size: 0.85rem;
  font-weight: 700;
  margin-bottom: -1px;
}
.code-label.python {
  background: rgba(255, 215, 0, 0.15);
  color: #ffd700;
}

/* TARJETAS PERSONALIZADAS */
.custom-card {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 12px;
  padding: 20px;
}
.border-purple {
  border-top: 3px solid #a855f7;
}
.border-orange {
  border-top: 3px solid #f97316;
}
.border-green {
  border-top: 3px solid #22c55e;
}
.border-white-10 {
  border: 1px solid rgba(255, 255, 255, 0.1);
}

.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 10px;
}

/* BUSCADOR */
.search-container {
  max-width: 100%;
}
.search-input :deep(.q-field__control) {
  background: rgba(15, 23, 42, 0.6) !important;
  border: 1px solid rgba(148, 163, 184, 0.3);
}

.glossary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(150px, 1fr));
  gap: 10px;
}
</style>
