<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        Un **Workspace** (ws) es simplemente una carpeta donde desarrollas tus proyectos de
        robótica. Pero no es una carpeta cualquiera; tiene una estructura interna rígida que ROS 2
        espera encontrar.
        <br /><br />
        Si pones tus archivos en el lugar incorrecto, ROS 2 será ciego a ellos.
      </TextBlock>
    </div>

    <!-- 1. ESTRUCTURA DE CARPETAS -->
    <div class="section-group">
      <SectionTitle>1. Los 4 Fantásticos</SectionTitle>
      <TextBlock>
        Un workspace compilado tiene siempre 4 carpetas principales. Al principio solo tienes una
        (`src`), las otras las crea el sistema automáticamente.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-3">
          <div class="folder-card src">
            <div class="icon">📂</div>
            <div class="name">src/</div>
            <div class="desc">SOURCE (Fuente). Aquí vive TU código. Es lo único que tocas.</div>
          </div>
        </div>
        <div class="col-12 col-md-3">
          <div class="folder-card build">
            <div class="icon">🔨</div>
            <div class="name">build/</div>
            <div class="desc">Archivos temporales de compilación. Basura técnica.</div>
          </div>
        </div>
        <div class="col-12 col-md-3">
          <div class="folder-card install">
            <div class="icon">🚀</div>
            <div class="name">install/</div>
            <div class="desc">
              El producto final. Aquí están los ejecutables y scripts de setup.
            </div>
          </div>
        </div>
        <div class="col-12 col-md-3">
          <div class="folder-card log">
            <div class="icon">📝</div>
            <div class="name">log/</div>
            <div class="desc">Reportes de errores. Si la compilación falla, mira aquí.</div>
          </div>
        </div>
      </div>
    </div>

    <!-- 2. CREANDO EL WORKSPACE -->
    <div class="section-group">
      <SectionTitle>2. Creando tu Primer Workspace</SectionTitle>
      <TextBlock>
        Vamos a crear la carpeta raíz y la subcarpeta <code>src</code>.
        <br />
        <em
          >Nota: Puedes llamarlo como quieras, pero <code>ros2_ws</code> es el estándar mundial.</em
        >
      </TextBlock>

      <CodeBlock
        title="Terminal"
        lang="bash"
        content="# 1. Ir a home
cd ~

# 2. Crear carpetas (la bandera -p crea padres e hijos a la vez)
mkdir -p ros2_ws/src

# 3. Entrar al workspace
cd ros2_ws

# 4. Verificar
ls
# Deberías ver SOLAMENTE la carpeta 'src'"
        :copyable="true"
      />
    </div>

    <!-- 3. LA REGLA DE ORO DEL COLCON -->
    <div class="section-group">
      <SectionTitle>3. La Regla de Oro del Colcon</SectionTitle>

      <AlertBlock type="danger" title="⚠️ DONDE EJECUTAR COLCON">
        NUNCA ejecutes <code>colcon build</code> dentro de la carpeta <code>src</code>. <br /><br />
        SIEMPRE debes estar en la <strong>RAÍZ DEL WORKSPACE</strong> (<code>~/ros2_ws</code>).
        <br />
        Si lo haces mal, crearás carpetas <code>build</code> anidadas horribles y todo fallará.
      </AlertBlock>

      <SplitBlock>
        <template #left>
          <div class="text-center q-pa-md bg-negative text-white rounded-borders">
            <div class="text-h6">❌ MAL</div>
            <code>~/ros2_ws/src $ colcon build</code>
          </div>
        </template>
        <template #right>
          <div class="text-center q-pa-md bg-positive text-white rounded-borders">
            <div class="text-h6">✅ BIEN</div>
            <code>~/ros2_ws $ colcon build</code>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. PAQUETES (PACKAGES) -->
    <div class="section-group">
      <SectionTitle>4. ¿Qué va dentro de src?</SectionTitle>
      <TextBlock>
        Dentro de <code>src</code> no puedes tirar archivos sueltos (`mi_script.py`). Debes
        organizarlos en <strong>Paquetes (Packages)</strong>. <br /><br />
        Un paquete es la unidad mínima de software en ROS 2. Es una carpeta que contiene:
        <br />
        1. <code>package.xml</code> (Manifiesto: ¿Quién soy? ¿Qué necesito?)
        <br />
        2. <code>setup.py</code> (Python) o <code>CMakeLists.txt</code> (C++) (Instrucciones de
        instalación)
      </TextBlock>

      <div class="tree-view q-pa-md q-my-md">
        <div>ros2_ws/</div>
        <div class="q-ml-md">├── build/</div>
        <div class="q-ml-md">├── install/</div>
        <div class="q-ml-md">├── log/</div>
        <div class="q-ml-md">└── src/</div>
        <div class="q-ml-xl">├── mi_robot_control/ (Paquete 1)</div>
        <div class="q-ml-xl">│ ├── package.xml</div>
        <div class="q-ml-xl">│ └── setup.py</div>
        <div class="q-ml-xl">└── mi_robot_vision/ (Paquete 2)</div>
        <div class="q-ml-xl">├── package.xml</div>
        <div class="q-ml-xl">└── CMakeLists.txt</div>
      </div>
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto de Arquitectura</SectionTitle>
      <StepsBlock
        :steps="[
          'Crea tu ~/ros2_ws/src si no lo tienes.',
          'Entra a la raíz: cd ~/ros2_ws',
          'Ejecuta \'colcon build\' (Aunque esté vacío).',
          'Observa cómo aparecen mágicamente las carpetas build, install y log.',
          'Explora la carpeta install: verás un archivo setup.bash. ¡Ese es el que activas con source!',
        ]"
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
import StepsBlock from 'components/content/StepsBlock.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3rem;
}

.folder-card {
  background: rgba(30, 41, 59, 0.5);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  height: 100%;
  text-align: center;
  transition: transform 0.2s;
}
.folder-card:hover {
  transform: translateY(-5px);
}

.folder-card.src {
  border-bottom: 4px solid #f59e0b;
} /* Amber */
.folder-card.build {
  border-bottom: 4px solid #64748b;
} /* Slate */
.folder-card.install {
  border-bottom: 4px solid #10b981;
} /* Emerald */
.folder-card.log {
  border-bottom: 4px solid #ef4444;
} /* Red */

.folder-card .icon {
  font-size: 2.5rem;
  margin-bottom: 10px;
}
.folder-card .name {
  font-weight: bold;
  font-size: 1.2rem;
  margin-bottom: 5px;
  color: #f8fafc;
  font-family: 'Fira Code', monospace;
}
.folder-card .desc {
  font-size: 0.85rem;
  color: #94a3b8;
}

.tree-view {
  background: #1e293b;
  border-radius: 8px;
  font-family: 'Fira Code', monospace;
  color: #a5f3fc;
  border: 1px solid #334155;
}
</style>
