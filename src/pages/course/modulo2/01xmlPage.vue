<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-red-4 text-weight-bold q-mb-sm">
          MÓDULO 2.1: EL ARQUITECTO RIGUROSO
        </div>

        <h1 class="hero-title">XML <span class="text-white">Básico</span></h1>

        <TextBlock>
          XML (eXtensible Markup Language) es el lenguaje que usamos para describir
          <strong>estructuras jerárquicas</strong>. En ROS 2, es la ley para dos cosas vitales:
          definir quién es tu robot (física, articulaciones) y definir qué necesita tu software
          (dependencias). A diferencia de Python, aquí no hay lógica; solo hay
          <strong>descripción</strong>.
        </TextBlock>
      </div>
    </section>

    <!-- 2. ANATOMÍA DE UNA ETIQUETA -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Anatomía Quirúrgica</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            Todo en XML son "Etiquetas" (Tags). Piensa en ellas como cajas dentro de cajas.
          </TextBlock>

          <ul class="tool-list q-mt-md">
            <li>
              🔒 <strong>Cierre Obligatorio:</strong> Todo lo que se abre
              <code>&lt;tag&gt;</code> debe cerrarse <code>&lt;/tag&gt;</code>.
            </li>
            <li>
              🔠 <strong>Case Sensitive:</strong> <code>&lt;Robot&gt;</code> no es lo mismo que
              <code>&lt;robot&gt;</code>.
            </li>
            <li>
              🌳 <strong>Raíz Única:</strong> Solo puede haber una etiqueta "padre" que englobe todo
              el archivo.
            </li>
          </ul>
        </div>

        <div class="col-12 col-md-7">
          <!-- VISUAL DISSECTION -->
          <div class="tool-card dissection-card relative-position q-pa-xl">
            <!-- ETIQUETA COMPLETA -->
            <div class="code-line row justify-center text-h5 font-mono items-center wrap">
              <span class="text-grey-6">&lt;</span>
              <span class="text-red-4 text-weight-bold element-hover q-mx-xs">cilindro</span>

              <span class="text-orange-4 attr-hover q-ml-sm">radio</span>
              <span class="text-grey-6">=</span>
              <span class="text-green-4">"10"</span>
              <span class="text-grey-6">&gt;</span>

              <span class="text-white q-mx-md content-hover">Motor Izquierdo</span>

              <span class="text-grey-6">&lt;/</span>
              <span class="text-red-4 text-weight-bold">cilindro</span>
              <span class="text-grey-6">&gt;</span>
            </div>

            <!-- ANNOTATIONS (LINEAS GUIAS) -->
            <div class="annotations q-mt-xl relative-position row justify-around text-center">
              <!-- Element Label -->
              <div class="column items-center">
                <q-icon name="arrow_upward" color="red-4" />
                <div class="text-caption text-red-4 text-weight-bold">Elemento</div>
              </div>

              <!-- Attribute Label -->
              <div class="column items-center">
                <q-icon name="arrow_upward" color="orange-4" />
                <div class="text-caption text-orange-4 text-weight-bold">Atributo</div>
              </div>

              <!-- Content Label -->
              <div class="column items-center">
                <q-icon name="arrow_upward" color="white" />
                <div class="text-caption text-white text-weight-bold">Contenido</div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. JERARQUÍA (TREE STRUCTURE) -->
    <div class="section-group self-stretch">
      <SectionTitle>2. El Árbol Genealógico</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            XML representa un árbol. No puedes tener etiquetas cruzadas.
            <br /><br />
            <strong>Correcto (Anidado):</strong>
            <br />El hijo vive completamente dentro del padre.
          </TextBlock>
          <div class="q-mt-lg">
            <AlertBlock type="danger" title="El error más común">
              Olvidar cerrar una etiqueta anidada rompe todo el archivo. Los simuladores de ROS
              (Gazebo/Rviz) simplemente no cargarán el robot.
            </AlertBlock>
          </div>
        </template>

        <template #right>
          <div class="tool-card tree-viz q-pa-lg">
            <!-- NIVEL 1: ROOT -->
            <div class="tree-box border-red">
              <div class="text-red-4 font-mono text-xs text-weight-bold">
                &lt;robot name="R2D2"&gt;
              </div>

              <div class="column q-gutter-sm q-ma-md border-left-guide q-pl-md">
                <!-- NIVEL 2: LINK -->
                <div class="tree-box border-orange bg-dark-subtle">
                  <div class="text-orange-4 font-mono text-xs text-weight-bold">
                    &lt;link name="cuerpo"&gt;
                  </div>
                  <div class="q-my-sm q-ml-md tree-box border-grey bg-black-subtle">
                    <span class="text-grey-4 font-mono text-xs"
                      >&lt;color&gt;Azul&lt;/color&gt;</span
                    >
                  </div>
                  <div class="text-orange-4 font-mono text-xs text-weight-bold">&lt;/link&gt;</div>
                </div>

                <!-- NIVEL 2: JOINT -->
                <div class="tree-box border-blue bg-dark-subtle">
                  <div class="text-blue-4 font-mono text-xs text-weight-bold">
                    &lt;joint name="cuello" /&gt;
                  </div>
                  <div class="text-caption text-grey-6 q-pa-xs text-right italic text-xs">
                    (Etiqueta auto-cerrada)
                  </div>
                </div>
              </div>

              <div class="text-red-4 font-mono text-xs text-weight-bold">&lt;/robot&gt;</div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. CASO REAL 1: PACKAGE.XML -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. En la Práctica: El Manifiesto (package.xml)</SectionTitle>
      <TextBlock>
        Cada vez que creas un paquete en ROS 2, se genera este archivo. Define la identidad de tu
        software. Si quieres que tu robot vea la cámara, debes declarar la dependencia aquí.
      </TextBlock>

      <div class="row q-mt-md">
        <div class="col-12">
          <div class="tool-card code-card q-pa-none">
            <div
              class="bg-red-9 text-white q-px-md q-py-sm text-subtitle2 flex justify-between items-center border-bottom-light"
            >
              <span class="font-mono text-weight-bold">package.xml</span>
              <q-icon name="description" />
            </div>
            <!-- CORREGIDO: lang & content -->
            <CodeBlock
              lang="xml"
              content='<?xml version="1.0"?>
<package format="3">
  <!-- Metadatos del Paquete -->
  <name>mi_robot_control</name>
  <version>0.0.1</version>
  <description>Controlador PID para el robot</description>

  <!-- Contacto Obligatorio -->
  <maintainer email="alex@robot.com">Alexander</maintainer>
  <license>Apache-2.0</license>

  <!-- Dependencias (LO MÁS IMPORTANTE) -->
  <depend>rclpy</depend>       <!-- Librería de ROS Python -->
  <depend>std_msgs</depend>    <!-- Mensajes estándar -->
  <exec_depend>ros2launch</exec_depend>

</package>'
            />
          </div>
        </div>
      </div>
    </div>

    <!-- 5. CASO REAL 2: URDF (Física) -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>4. En la Práctica: Física del Robot (URDF)</SectionTitle>
      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-6">
          <TextBlock>
            El <strong>Unified Robot Description Format (URDF)</strong> es puro XML. Aquí es donde
            XML brilla (y duele). Usamos <strong>Atributos</strong> para definir propiedades
            físicas.
          </TextBlock>
          <ul class="tool-list q-mt-md">
            <li>
              En <code>package.xml</code> usamos contenido:
              <code>&lt;name&gt;texto&lt;/name&gt;</code>
            </li>
            <li>En URDF usamos atributos numéricos: <code>radius="0.5"</code></li>
          </ul>
        </div>
        <div class="col-12 col-md-6">
          <div class="tool-card code-card q-pa-none border-blue">
            <div
              class="bg-blue-9 text-white q-px-md q-py-sm text-subtitle2 flex justify-between items-center border-bottom-light"
            >
              <span class="font-mono text-weight-bold">robot_model.urdf</span>
              <q-icon name="precision_manufacturing" />
            </div>
            <!-- CORREGIDO: lang & content -->
            <CodeBlock
              lang="xml"
              content='<robot name="brazo_robotico">

  <!-- Eslabón Base -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <!-- Articulación (Joint) -->
  <joint name="base_to_brazo" type="revolute">
    <parent link="base_link"/>
    <child link="brazo_1"/>
    <limit lower="-1.57" upper="1.57"/>
  </joint>

</robot>'
            />
          </div>
        </div>
      </div>
    </div>

    <!-- 6. PRO TIP: COMENTARIOS -->
    <div class="section-group self-stretch q-mb-xl">
      <div class="tool-card bg-slate-800 q-pa-lg rounded-borders border-dashed text-center">
        <q-icon name="lightbulb" size="md" color="yellow-4" class="q-mb-sm" />
        <div class="text-h6 text-white">El Arte de Ignorar</div>
        <p class="text-grey-4 q-mb-md">
          En XML, los comentarios son extraños. No usamos <code>#</code> ni <code>//</code>. Usamos
          esta secuencia críptica:
        </p>
        <div class="row justify-center">
          <div class="bg-black q-px-xl q-py-md rounded-borders font-mono text-grey-5 border-light">
            <span class="text-green-4">&lt;!--</span> Esto es un comentario invisible
            <span class="text-green-4">--&gt;</span>
          </div>
        </div>
        <p class="text-caption q-mt-sm text-red-3">
          ¡Cuidado! No puedes poner -- (doble guion) dentro de un comentario. Rompe el parser.
        </p>
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
</script>

<style scoped>
/* --- ESTILOS MAESTROS --- */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}

.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at center, rgba(239, 68, 68, 0.15), transparent 60%),
    rgba(15, 23, 42, 0.8);
  backdrop-filter: blur(20px);
  border-radius: 24px;
  border: 1px solid rgba(148, 163, 184, 0.2);
  text-align: center;
}

.hero-title {
  font-size: 3rem;
  font-weight: 800;
  margin: 0 0 1.5rem 0;
  line-height: 1.1;
  color: #f8fafc;
}

/* TOOL CARDS */
.tool-card {
  height: 100%;
  border-radius: 16px;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.05);
}

/* DISSECTION CARD */
.dissection-card {
  border-top: 4px solid #ef4444;
  background: #0f172a;
}

.element-hover:hover {
  text-shadow: 0 0 10px rgba(248, 113, 113, 0.8);
  cursor: help;
}
.attr-hover:hover {
  text-shadow: 0 0 10px rgba(251, 146, 60, 0.8);
  cursor: help;
}
.content-hover:hover {
  text-shadow: 0 0 10px rgba(255, 255, 255, 0.8);
  cursor: help;
}

/* TREE VIZ */
.tree-box {
  padding: 8px 12px;
  border-radius: 6px;
  border-left: 3px solid;
  transition: background 0.2s;
}
.tree-box:hover {
  background: rgba(255, 255, 255, 0.05);
}

.border-red {
  border-left-color: #ef4444;
}
.border-orange {
  border-left-color: #f97316;
}
.border-blue {
  border-left-color: #3b82f6;
}
.border-grey {
  border-left-color: #64748b;
}
.border-left-guide {
  border-left: 1px solid rgba(255, 255, 255, 0.1);
}

.bg-dark-subtle {
  background: rgba(0, 0, 0, 0.2);
}
.bg-black-subtle {
  background: rgba(0, 0, 0, 0.4);
}

.italic {
  font-style: italic;
}
.text-xs {
  font-size: 0.85rem;
}

/* CODE CARD */
.tool-card.code-card {
  overflow: hidden;
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.tool-card.border-blue {
  border-color: #3b82f6;
}

.border-dashed {
  border: 1px dashed rgba(255, 255, 255, 0.2);
}
.border-bottom-light {
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
}

.font-mono {
  font-family: 'Fira Code', monospace;
}
.tool-list {
  list-style: none;
  padding: 0;
  color: #cbd5e1;
}
.tool-list li {
  margin-bottom: 8px;
  font-size: 0.95rem;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .code-line {
    font-size: 1rem;
  }
}
</style>
