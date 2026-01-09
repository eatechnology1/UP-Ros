<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        Imagina que tienes una caja de herramientas guardada en el sótano. Si quieres usar el
        martillo en la cocina, tienes que ir a buscarlo o decirle a alguien dónde está.
        <br /><br />
        Las <strong>Variables de Entorno</strong> son como notas adhesivas que le dicen a Linux
        dónde están guardadas tus herramientas (programas) y configuraciones importantes.
      </TextBlock>
    </div>

    <!-- 1. ¿QUÉ SON? -->
    <div class="section-group">
      <SectionTitle>1. El Mapa del Tesoro: PATH</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Cuando escribes un comando como <code>ls</code> o <code>python</code>, Linux no busca en
            todo el disco duro (tardaría horas). Solo busca en las carpetas listadas en una variable
            llamada <code>PATH</code>. <br /><br />
            Si tu programa no está en el <code>PATH</code>, la terminal te dirá:
            <em>"Command not found"</em>.
          </TextBlock>
        </template>
        <template #right>
          <div class="bg-dark q-pa-md rounded-borders">
            <div class="text-caption text-grey-5 q-mb-xs">Ver tus variables:</div>
            <CodeBlock lang="bash" content="printenv | grep PATH" :copyable="true" />
            <div class="text-caption text-grey-5 q-mt-sm">
              Verás una lista de rutas separadas por dos puntos (:).
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 2. SOURCING EN ROS 2 -->
    <div class="section-group">
      <SectionTitle>2. El Ritual del "Source"</SectionTitle>

      <AlertBlock type="warning" title="Concepto Clave">
        ROS 2 no se instala en las carpetas estándar de Linux. Se esconde en
        <code>/opt/ros/jazzy</code>.
        <br />
        Por eso, cada vez que abres una terminal nueva, Linux NO sabe que ROS existe.
      </AlertBlock>

      <TextBlock>
        Para "activar" ROS, ejecutamos un script especial que inyecta las variables necesarias. A
        esto le llamamos <strong>"Sourcing"</strong>.
      </TextBlock>

      <div class="q-my-md">
        <CodeBlock
          title="El comando mágico"
          lang="bash"
          content="source /opt/ros/jazzy/setup.bash"
          :copyable="true"
        />
      </div>

      <TextBlock>
        Si intentas usar comandos de ROS (como <code>ros2 topic list</code>) sin hacer esto,
        fallarán. Al hacer <code>source</code>, estás diciendo: "Hey Linux, agrega todas las
        herramientas de esta carpeta a tu PATH temporalmente".
      </TextBlock>
    </div>

    <!-- 3. AUTOMATIZACIÓN (.BASHRC) -->
    <div class="section-group">
      <SectionTitle>3. Automatización (.bashrc)</SectionTitle>
      <TextBlock>
        ¿Te imaginas tener que escribir ese comando cada vez que abres una terminal? Sería una
        tortura. Para eso existe el archivo <code>.bashrc</code>. <br /><br />
        El <code>.bashrc</code> es un script que se ejecuta <strong>automáticamente</strong> cada
        vez que abres una ventana nueva de terminal. Si ponemos el comando <code>source</code> ahí
        dentro, ROS estará siempre listo.
      </TextBlock>

      <StepsBlock
        :steps="[
          'Asegúrate de estar en tu carpeta Home: cd ~',
          'Abre el archivo oculto .bashrc con un editor (nano): nano .bashrc',
          'Ve al final del archivo (baja con las flechas).',
          'Escribe al final: source /opt/ros/jazzy/setup.bash',
          'Guarda con Ctrl+O, Enter. Sal con Ctrl+X.',
          'Para aplicar cambios en ESTA terminal: source ~/.bashrc',
        ]"
      />

      <div class="q-mt-md">
        <AlertBlock type="success" title="Truco Pro">
          Puedes hacer todo lo anterior con un solo comando:
          <br />
          <code>echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc</code>
        </AlertBlock>
      </div>
    </div>

    <!-- 4. VARIABLES ESPECÍFICAS DE ROS -->
    <div class="section-group">
      <SectionTitle>4. Variables Críticas de ROS</SectionTitle>
      <TextBlock>
        Además del PATH, ROS usa variables para saber cómo comunicarse entre robots.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="env-card">
            <div class="text-h6 text-accent">ROS_DOMAIN_ID</div>
            <p>
              Es como el "Canal de Radio". Si dos robots tienen el mismo ID (ej. 0), se hablan entre
              sí. Si quieres aislarlos, cambias el ID.
            </p>
            <CodeBlock lang="bash" content="export ROS_DOMAIN_ID=32" />
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="env-card">
            <div class="text-h6 text-secondary">ROS_LOCALHOST_ONLY</div>
            <p>
              Por seguridad, a veces queremos que el robot solo hable consigo mismo y no salga a la
              red WiFi.
            </p>
            <CodeBlock lang="bash" content="export ROS_LOCALHOST_ONLY=1" />
          </div>
        </div>
      </div>
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto de Configuración</SectionTitle>
      <TextBlock> Verifica que tu entorno está listo para trabajar. </TextBlock>

      <CodeBlock
        title="Diagnóstico"
        lang="bash"
        content="# 1. Verifica si ROS está cargado
printenv | grep ROS

# Deberías ver líneas como:
# ROS_DISTRO=jazzy
# ROS_VERSION=2"
        :copyable="true"
      />

      <div class="q-mt-md text-center">
        <p class="text-grey-4">Si no sale nada, ¡vuelve al paso del <code>.bashrc</code>!</p>
      </div>
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

.env-card {
  background: rgba(30, 41, 59, 0.5);
  border-left: 4px solid #d8b4fe; /* Morado suave */
  border-radius: 8px;
  padding: 1.5rem;
  height: 100%;
}
.env-card .text-h6 {
  font-family: 'Fira Code', monospace;
  margin-bottom: 0.5rem;
}
.env-card p {
  color: #cbd5e1;
  font-size: 0.95rem;
  margin-bottom: 1rem;
}
</style>
