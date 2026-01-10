<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        La <strong>CLI (Command Line Interface)</strong> de ROS 2 es tu panel de control. A
        diferencia de programas con botones bonitos, aquí escribimos órdenes directas al núcleo del
        robot. <br /><br />
        Para esta práctica, usaremos el simulador más famoso de la historia:
        <strong>Turtlesim</strong>.
      </TextBlock>
    </div>

    <!-- 0. PREPARACIÓN -->
    <div class="section-group">
      <SectionTitle>0. Invocando a la Tortuga</SectionTitle>
      <TextBlock>
        Necesitamos dos terminales. Una para el simulador (la tortuga) y otra para controlarla (el
        teclado).
      </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="terminal-window">
            <div class="terminal-header">Terminal 1 (Simulador)</div>
            <CodeBlock lang="bash" content="ros2 run turtlesim turtlesim_node" />
            <p class="text-caption q-mt-sm">Aparecerá una ventana azul con una tortuga.</p>
          </div>
        </template>
        <template #right>
          <div class="terminal-window">
            <div class="terminal-header">Terminal 2 (Teleop)</div>
            <CodeBlock lang="bash" content="ros2 run turtlesim turtle_teleop_key" />
            <p class="text-caption q-mt-sm">Usa las flechas del teclado AQUÍ para moverla.</p>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 1. NODOS -->
    <div class="section-group">
      <SectionTitle>1. Espiando Nodos (ros2 node)</SectionTitle>
      <TextBlock>
        Un nodo es un proceso (programa) que hace algo. Queremos ver quién está vivo en la red.
      </TextBlock>

      <CodeBlock title="Listar Nodos" lang="bash" content="ros2 node list" />

      <AlertBlock type="info" title="Salida Esperada">
        /turtlesim &lt;-- El simulador gráfico
        <br />
        /teleop_turtle &lt;-- Tu teclado
      </AlertBlock>

      <TextBlock class="q-mt-md">
        Si quieres saber todo sobre un nodo (qué publica, a qué se suscribe), úsalo con
        <code>info</code>.
      </TextBlock>
      <CodeBlock lang="bash" content="ros2 node info /turtlesim" />
    </div>

    <!-- 2. TÓPICOS -->
    <div class="section-group">
      <SectionTitle>2. Escuchando Conversaciones (ros2 topic)</SectionTitle>
      <TextBlock>
        Los nodos hablan a través de <strong>Tópicos</strong> (canales de chat). El nodo `/teleop`
        le grita comandos de velocidad al nodo `/turtlesim`.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="cmd-card">
            <div class="text-h6 text-primary">ros2 topic list</div>
            <p>Muestra todos los canales activos.</p>
            <code>/turtle1/cmd_vel</code>
            <code>/turtle1/pose</code>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="cmd-card">
            <div class="text-h6 text-accent">ros2 topic echo [tema]</div>
            <p>Espía los datos en tiempo real.</p>
            <code>ros2 topic echo /turtle1/pose</code>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="success" title="Reto: Publicar Manualmente">
          Puedes hacerte pasar por el teclado y enviar un comando directo desde la terminal.
          <br /><br />
          <code
            >ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y:
            0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"</code
          >
          <br /><br />
          ¡La tortuga debería moverse y girar una sola vez!
        </AlertBlock>
      </div>
    </div>

    <!-- 3. RQT_GRAPH -->
    <div class="section-group">
      <SectionTitle>3. La Visión de Dios (rqt_graph)</SectionTitle>
      <TextBlock>
        Ver texto en la terminal está bien, pero ver el mapa de conexiones es mejor. ROS 2 incluye
        una herramienta visual llamada <strong>rqt_graph</strong>.
      </TextBlock>

      <CodeBlock lang="bash" content="rqt_graph" />

      <div class="graph-visual q-my-md text-center">
        <!-- Simulación visual simple del grafo -->
        <div class="node-oval">/teleop_turtle</div>
        <div class="arrow-right">
          <span>/turtle1/cmd_vel</span>
          ➞
        </div>
        <div class="node-oval">/turtlesim</div>
      </div>

      <TextBlock>
        Verás un óvalo (Nodo) enviando una flecha (Tópico) a otro óvalo. Si la flecha está rota o no
        conecta, ¡ahí está tu bug!
      </TextBlock>
    </div>

    <!-- 4. INTERFACES -->
    <div class="section-group">
      <SectionTitle>4. El Diccionario (ros2 interface)</SectionTitle>
      <TextBlock>
        Para hablar, necesitas saber el idioma. Los tópicos usan <strong>Tipos de Mensaje</strong>.
        Por ejemplo, el tópico `/turtle1/pose` usa el mensaje `turtlesim/msg/Pose`. ¿Qué hay dentro
        de ese mensaje?
      </TextBlock>

      <CodeBlock
        title="Inspeccionando Estructuras"
        lang="bash"
        content="ros2 interface show turtlesim/msg/Pose"
      />

      <div class="bg-dark q-pa-sm rounded-borders text-caption text-grey-4 q-mt-xs">
        float32 x<br />
        float32 y<br />
        float32 theta<br />
        float32 linear_velocity<br />
        float32 angular_velocity
      </div>
    </div>

    <!-- RETO FINAL -->
    <div class="section-group">
      <SectionTitle>🏆 Reto Hacker</SectionTitle>
      <StepsBlock
        :steps="[
          'Ejecuta la tortuga y el teleop.',
          'Abre una 3ra terminal.',
          'Usa \'ros2 topic echo /turtle1/pose\'.',
          'Mueve la tortuga y observa cómo cambian los números (X, Y) en tiempo real.',
          '¡Acabas de interceptar la telemetría del robot!',
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

.terminal-window {
  background: #1e293b;
  border-radius: 8px;
  padding: 10px;
  border: 1px solid #334155;
}
.terminal-header {
  font-size: 0.8rem;
  color: #94a3b8;
  margin-bottom: 8px;
  border-bottom: 1px solid #334155;
  padding-bottom: 4px;
}

.cmd-card {
  background: rgba(30, 41, 59, 0.5);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  height: 100%;
}
.cmd-card code {
  display: block;
  background: rgba(0, 0, 0, 0.3);
  padding: 4px 8px;
  border-radius: 4px;
  margin-top: 8px;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
  color: #a5f3fc;
}

/* Visualización Grafo */
.graph-visual {
  display: flex;
  justify-content: center;
  align-items: center;
  gap: 10px;
  background: white;
  padding: 20px;
  border-radius: 8px;
  color: black;
}
.node-oval {
  border: 2px solid black;
  border-radius: 50px; /* Oval */
  padding: 10px 20px;
  font-weight: bold;
  background: #e2e8f0;
}
.arrow-right {
  display: flex;
  flex-direction: column;
  align-items: center;
  font-size: 1.5rem;
  color: #ef4444; /* Rojo como en rqt */
}
.arrow-right span {
  font-size: 0.8rem;
  color: black;
}
</style>
