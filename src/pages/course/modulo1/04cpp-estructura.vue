<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">
          MÓDULO 1.4: ARQUITECTURA C++
        </div>

        <h1 class="hero-title">Headers y <span class="text-primary">Sources</span></h1>

        <TextBlock>
          En proyectos profesionales de robótica, no escribimos todo el código en un solo lugar.
          Aprendemos a separar las <strong>Promesas</strong> (Archivos .hpp) de las
          <strong>Acciones</strong> (Archivos .cpp). Esta estructura es obligatoria para crear nodos
          de ROS 2 limpios y compilables.
        </TextBlock>
      </div>
    </section>

    <!-- 2. LA METÁFORA: MENÚ VS COCINA -->
    <div class="section-group self-stretch">
      <SectionTitle>1. La Gran División (.hpp vs .cpp)</SectionTitle>

      <div class="row q-col-gutter-lg items-stretch">
        <!-- HEADER FILE -->
        <div class="col-12 col-md-6">
          <div class="tool-card file-type header-file relative-position full-height">
            <div class="file-badge bg-orange-9 text-white shadow-2">.hpp / .h</div>

            <div class="q-pa-lg text-center">
              <q-icon name="menu_book" color="orange-4" size="4rem" class="q-mb-md" />
              <h3 class="text-h5 text-white q-my-sm">El "Menú" (Declaración)</h3>
              <p class="text-body2 text-grey-4">
                Le dice al compilador <strong>QUÉ</strong> existe. <br />Listas las variables y
                funciones, pero no escribes su lógica. Es el contrato público de tu robot.
              </p>
            </div>

            <div class="code-snippet q-px-md q-pb-md col-grow flex column justify-end">
              <!-- CORREGIDO: lang & content -->
              <CodeBlock
                lang="cpp"
                content="class Robot {
  public:
    void mover(); // Solo prometo que me muevo
  private:
    int bateria;
};"
              />
            </div>
          </div>
        </div>

        <!-- SOURCE FILE -->
        <div class="col-12 col-md-6">
          <div class="tool-card file-type source-file relative-position full-height">
            <div class="file-badge bg-blue-9 text-white shadow-2">.cpp</div>

            <div class="q-pa-lg text-center">
              <q-icon name="soup_kitchen" color="blue-4" size="4rem" class="q-mb-md" />
              <h3 class="text-h5 text-white q-my-sm">La "Cocina" (Implementación)</h3>
              <p class="text-body2 text-grey-4">
                Le dice al compilador <strong>CÓMO</strong> funciona. <br />Aquí está la lógica, las
                matemáticas y el código sucio. Nadie necesita ver esto para usar tu robot.
              </p>
            </div>

            <div class="code-snippet q-px-md q-pb-md col-grow flex column justify-end">
              <!-- CORREGIDO: lang & content -->
              <CodeBlock
                lang="cpp"
                content='#include "robot.hpp"

void Robot::mover() {
  this->bateria -= 10;
  printf("Avanzando...");
}'
              />
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. EL ALCANCE (SCOPE) Y :: -->
    <div class="section-group self-stretch">
      <SectionTitle>2. El Operador de Pertenencia (::)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Cuando estás en el archivo <code>.cpp</code>, estás fuera de la clase. Para definir una
            función que prometiste en el <code>.hpp</code>, debes usar el apellido de la clase.
            <br /><br />
            El operador <code>::</code> significa <strong>"Pertenece a"</strong>. <br /><br />
            Sin esto, C++ pensará que <code>mover()</code> es una función suelta que no tiene nada
            que ver con tu robot.
          </TextBlock>
        </template>

        <template #right>
          <div class="nano-terminal q-pa-md font-mono text-caption rounded-borders shadow-2">
            <div class="text-grey-5">// En robot.cpp</div>
            <br />
            <div class="text-red-4">void mover() { ... }</div>
            <div class="text-grey-5 q-mb-md border-left-red q-pl-sm">
              ❌ Error: ¿Quién se mueve? ¿El mundo? El compilador no sabe que esto es del robot.
            </div>

            <div class="text-green-4">
              void Robot<span class="text-yellow-4">::</span>mover() { ... }
            </div>
            <div class="text-grey-5 border-left-green q-pl-sm">
              ✅ Correcto: La función 'mover' que <strong>pertenece a</strong> la clase 'Robot'.
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. PUNTEROS INTELIGENTES -->
    <div class="section-group self-stretch">
      <SectionTitle>3. Memoria Moderna: Smart Pointers</SectionTitle>
      <AlertBlock type="warning" title="💀 Olvida el 'new' y 'delete'">
        En C++ antiguo (y en la universidad) te enseñan a gestionar memoria manualmente.
        <strong>En ROS 2 eso está prohibido.</strong>
        Usamos "Punteros Inteligentes" que borran la memoria automáticamente cuando ya no se usan.
      </AlertBlock>

      <div class="row q-col-gutter-lg q-mt-sm items-stretch">
        <div class="col-12 col-md-6">
          <div class="tool-card concept-card bg-slate-800 full-height column">
            <div class="text-subtitle1 text-accent text-weight-bold q-mb-sm">std::shared_ptr</div>
            <p class="text-grey-4 flex-grow">
              Es como una correa compartida. Varios sistemas pueden "sostener" al mismo robot. El
              robot no se destruye hasta que el último sistema suelta la correa.
              <br /><strong class="text-white">Es el estándar en ROS 2.</strong>
            </p>
            <div class="q-mt-auto">
              <!-- CORREGIDO: lang & content -->
              <CodeBlock lang="cpp" content="auto nodo = std::make_shared<MiRobot>();" />
            </div>
          </div>
        </div>

        <div class="col-12 col-md-6">
          <div class="tool-card concept-card bg-slate-800 full-height column">
            <div class="text-subtitle1 text-purple-4 text-weight-bold q-mb-sm">std::unique_ptr</div>
            <p class="text-grey-4 flex-grow">
              Propiedad exclusiva. "Este sensor es mío y de nadie más". Si intentas copiarlo, el
              compilador te grita. Se usa para hardware exclusivo o drivers.
            </p>
            <div class="q-mt-auto">
              <!-- CORREGIDO: lang & content -->
              <CodeBlock
                lang="cpp"
                content="std::unique_ptr<Sensor> s = std::make_unique<Sensor>();"
              />
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. ARQUITECTURA DE CARPETAS ROS 2 -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>4. ¿Dónde guardo mis archivos?</SectionTitle>
      <div class="tool-card file-tree-card row items-center justify-center">
        <div class="col-12 col-md-8">
          <div class="text-caption text-grey-5 q-mb-md text-center font-mono">
            ESTRUCTURA TÍPICA DE PAQUETE C++ (ament_cmake)
          </div>

          <ul class="file-tree font-mono text-body2">
            <li><q-icon name="folder" color="blue-4" /> my_robot_cpp/</li>
            <li>
              <ul>
                <li>
                  <q-icon name="description" color="grey-6" /> CMakeLists.txt
                  <span class="text-grey-6 text-caption q-ml-sm">(La Receta)</span>
                </li>
                <li><q-icon name="description" color="grey-6" /> package.xml</li>

                <!-- INCLUDE FOLDER -->
                <li class="bg-highlight-orange rounded-borders q-pa-sm q-my-xs">
                  <q-icon name="folder_open" color="orange-4" /> include/my_robot_cpp/
                  <ul>
                    <li>
                      <q-icon name="description" color="orange-3" />
                      <strong>robot_brain.hpp</strong>
                      <span class="text-accent text-caption q-ml-sm">← Headers aquí</span>
                    </li>
                  </ul>
                </li>

                <!-- SRC FOLDER -->
                <li class="bg-highlight-blue rounded-borders q-pa-sm q-my-xs">
                  <q-icon name="folder_open" color="blue-4" /> src/
                  <ul>
                    <li>
                      <q-icon name="description" color="blue-3" /> <strong>robot_brain.cpp</strong>
                      <span class="text-accent text-caption q-ml-sm">← Código aquí</span>
                    </li>
                    <li>
                      <q-icon name="description" color="blue-3" /> main.cpp
                      <span class="text-grey-5 text-caption q-ml-sm">(Ejecutable)</span>
                    </li>
                  </ul>
                </li>
              </ul>
            </li>
          </ul>
        </div>
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
    radial-gradient(circle at center, rgba(59, 130, 246, 0.15), transparent 60%),
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

/* Header vs Source Cards */
.tool-card.file-type {
  display: flex;
  flex-direction: column;
  transition: transform 0.3s;
}
.tool-card.file-type:hover {
  transform: translateY(-5px);
}
.tool-card.header-file {
  border-top: 4px solid #f97316;
} /* Orange */
.tool-card.source-file {
  border-top: 4px solid #3b82f6;
} /* Blue */

.file-badge {
  position: absolute;
  top: 0;
  right: 0;
  padding: 6px 16px;
  border-bottom-left-radius: 12px;
  font-family: 'Fira Code', monospace;
  font-weight: bold;
  z-index: 10;
}

.concept-card {
  padding: 24px;
}
.flex-grow {
  flex-grow: 1;
}

/* FILE TREE */
.tool-card.file-tree-card {
  padding: 32px;
  border-top: 4px solid #94a3b8;
}
.file-tree {
  list-style: none;
  padding-left: 0;
  color: #e2e8f0;
  margin: 0;
}
.file-tree ul {
  list-style: none;
  padding-left: 24px;
  border-left: 1px solid #475569;
  margin-top: 8px;
}
.file-tree li {
  display: flex;
  align-items: center;
  gap: 8px;
}

.bg-highlight-orange {
  background: rgba(251, 146, 60, 0.08);
  border: 1px dashed rgba(251, 146, 60, 0.3);
}
.bg-highlight-blue {
  background: rgba(59, 130, 246, 0.08);
  border: 1px dashed rgba(59, 130, 246, 0.3);
}

/* TERMINAL SIMULATION */
.nano-terminal {
  background-color: #0f172a;
  border: 1px solid #334155;
}
.border-left-red {
  border-left: 3px solid #f87171;
}
.border-left-green {
  border-left: 3px solid #4ade80;
}

.font-mono {
  font-family: 'Fira Code', monospace;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
