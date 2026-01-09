<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        Ya sabes que necesitas hacer <code>source</code> para que Linux encuentre los comandos. Pero
        en ROS 2, no trabajamos con una sola fuente de software. Trabajamos con **Capas**.
        <br /><br />
        Tener un entorno bien configurado (`.bashrc`) es la diferencia entre odiar ROS y amarlo.
      </TextBlock>
    </div>

    <!-- 1. UNDERLAY VS OVERLAY -->
    <div class="section-group">
      <SectionTitle>1. Teoría del Pastel: Underlay y Overlay</SectionTitle>
      <TextBlock> Imagina que el software de tu robot es un pastel de varias capas. </TextBlock>

      <div class="layer-container q-my-md">
        <!-- Capa Superior -->
        <div class="layer overlay">
          <div class="layer-label">OVERLAY (Tu Workspace)</div>
          <div class="layer-desc">~/ros2_ws/src</div>
          <p>
            Aquí está TU código. Si hay un nodo aquí con el mismo nombre que abajo,
            <strong>este gana</strong>.
          </p>
        </div>

        <!-- Flecha -->
        <div class="layer-arrow">⬇ se apoya sobre ⬇</div>

        <!-- Capa Inferior -->
        <div class="layer underlay">
          <div class="layer-label">UNDERLAY (Instalación Base)</div>
          <div class="layer-desc">/opt/ros/jazzy</div>
          <p>Aquí están las librerías estándar (rclpy, std_msgs, nav2). Es la base sólida.</p>
        </div>
      </div>

      <AlertBlock type="warning" title="El Orden Importa">
        Debes cargar (source) primero el Underlay y LUEGO el Overlay. Si lo haces al revés, tu
        entorno podría romperse.
      </AlertBlock>
    </div>

    <!-- 2. AUTOMATIZACIÓN DEL BASHRC -->
    <div class="section-group">
      <SectionTitle>2. El .bashrc Perfecto</SectionTitle>
      <TextBlock>
        Vamos a configurar tu archivo <code>~/.bashrc</code> para que cargue ambas capas
        automáticamente y añada atajos para no escribir tanto.
      </TextBlock>

      <CodeBlock
        title="~/.bashrc recomendado"
        lang="bash"
        content="# --- SECCIÓN ROS 2 ---

# 1. Cargar el Underlay (Base)
source /opt/ros/jazzy/setup.bash

# 2. Cargar mi Overlay (Mi código)
# (Solo si el archivo existe, para evitar errores si borras la carpeta)
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# 3. Configuración de Red (Opcional pero recomendada)
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# --- FIN SECCIÓN ROS 2 ---"
        :copyable="true"
      />

      <div class="q-mt-md">
        <TextBlock>
          Copia esto y pégalo al final de tu archivo <code>.bashrc</code> (<code
            >nano ~/.bashrc</code
          >). Recuerda recargar la terminal con <code>source ~/.bashrc</code>.
        </TextBlock>
      </div>
    </div>

    <!-- 3. ALIAS DE PODER -->
    <div class="section-group">
      <SectionTitle>3. Alias para Ingenieros Productivos</SectionTitle>
      <TextBlock>
        En ROS 2 escribimos comandos muy largos repetidamente. Vamos a crear "hechizos cortos"
        (alias) para ahorrar tiempo.
      </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="text-subtitle1 text-primary">Añade esto a tu .bashrc</div>
          <CodeBlock
            lang="bash"
            content="alias cb='colcon build --symlink-install'
alias s='source install/setup.bash'
alias rt='ros2 topic list'
alias rn='ros2 node list'"
            :copyable="true"
          />
        </template>
        <template #right>
          <div class="text-subtitle1 text-secondary">¿Qué hacen?</div>
          <ul class="alias-list">
            <li>
              <strong>cb:</strong> Compila todo tu workspace. (La bandera symlink permite cambiar
              Python sin recompilar).
            </li>
            <li><strong>s:</strong> Recarga tu entorno después de compilar (Vital).</li>
            <li><strong>rt:</strong> Lista los tópicos rápidos.</li>
          </ul>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. CHECK DE ENTORNO -->
    <div class="section-group">
      <SectionTitle>4. Diagnóstico de Variables</SectionTitle>
      <TextBlock>
        ¿Cómo sabes si tu Overlay está cargado correctamente encima del Underlay? Revisando la
        variable <code>ROS_PACKAGE_PATH</code>.
      </TextBlock>

      <CodeBlock title="Verificando Capas" lang="bash" content="printenv ROS_PACKAGE_PATH" />

      <AlertBlock type="success" title="Salida Correcta">
        Deberías ver <strong>DOS</strong> rutas separadas por dos puntos (:).
        <br />
        <code>/home/user/ros2_ws/install/share : /opt/ros/jazzy/share</code>
        <br /><br />
        Si solo ves <code>/opt/ros...</code>, significa que tu Workspace no está cargado.
      </AlertBlock>
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto de Configuración</SectionTitle>
      <StepsBlock
        :steps="[
          'Edita tu .bashrc con nano.',
          'Agrega el sourcing del Underlay (/opt/ros/jazzy).',
          'Agrega el alias cb (colcon build).',
          'Guarda, cierra y abre una NUEVA terminal.',
          'Escribe \'cb\'. Si te dice \'colcon: command not found\', es que no tienes instalado colcon (lo veremos luego) o falló el sourcing.',
          'Si dice \'command not found\' pero para cb, es que el alias no cargó.',
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

/* Estilos para el diagrama de Capas */
.layer-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 10px;
  max-width: 600px;
  margin: 0 auto;
}

.layer {
  width: 100%;
  padding: 20px;
  border-radius: 12px;
  text-align: center;
  position: relative;
  transition: transform 0.3s;
}
.layer:hover {
  transform: scale(1.02);
}

.layer.overlay {
  background: rgba(16, 185, 129, 0.15); /* Verde suave */
  border: 2px solid #10b981;
  z-index: 2;
}

.layer.underlay {
  background: rgba(59, 130, 246, 0.15); /* Azul suave */
  border: 2px solid #3b82f6;
  z-index: 1;
}

.layer-label {
  font-weight: 800;
  font-size: 1.2rem;
  letter-spacing: 1px;
  margin-bottom: 5px;
}
.layer.overlay .layer-label {
  color: #34d399;
}
.layer.underlay .layer-label {
  color: #60a5fa;
}

.layer-desc {
  font-family: 'Fira Code', monospace;
  background: rgba(0, 0, 0, 0.3);
  display: inline-block;
  padding: 4px 10px;
  border-radius: 4px;
  margin-bottom: 10px;
  font-size: 0.9rem;
}

.layer-arrow {
  color: #94a3b8;
  font-weight: bold;
  font-size: 0.9rem;
  letter-spacing: 2px;
}

.alias-list {
  color: #cbd5e1;
  line-height: 2;
}
.alias-list strong {
  color: #fbbf24; /* Amber */
  font-family: 'Fira Code', monospace;
}
</style>
