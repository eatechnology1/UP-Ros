<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        Imagina que estás escribiendo una novela. Quieres probar un final donde el protagonista
        muere, pero no estás seguro de si gustará.
        <br /><br />
        En lugar de borrar tu borrador actual, creas una copia paralela (una dimensión alternativa).
        En esa dimensión, matas al protagonista. Si te gusta, fusionas esa dimensión con la real. Si
        no, simplemente destruyes esa dimensión y aquí no ha pasado nada.
        <br /><br />
        En Git, estas dimensiones se llaman <strong>Ramas (Branches)</strong>.
      </TextBlock>
    </div>

    <!-- 1. CREAR Y CAMBIAR (SWITCH) -->
    <div class="section-group">
      <SectionTitle>1. El Multiverso (Branch & Switch)</SectionTitle>
      <TextBlock>
        Por defecto, vives en la rama <code>main</code> (la línea temporal sagrada). Nunca deberías
        programar directamente ahí si estás experimentando.
      </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="text-subtitle2 text-primary">Comandos Antiguos (Confusos)</div>
          <CodeBlock lang="bash" content="git checkout -b nueva-rama" />
          <p class="text-caption text-grey">Checkout servía para todo, por eso confundía.</p>
        </template>
        <template #right>
          <div class="text-subtitle2 text-accent">Comandos Modernos (Claros)</div>
          <CodeBlock
            lang="bash"
            content="# 1. Crear rama
git branch experimento-lidar

# 2. Cambiarse a ella
git switch experimento-lidar

# Truco: Crear y cambiar a la vez
git switch -c experimento-lidar"
          />
        </template>
      </SplitBlock>
    </div>

    <!-- 2. TRABAJAR EN PARALELO -->
    <div class="section-group">
      <SectionTitle>2. Trabajando en Aislamiento</SectionTitle>

      <!-- Diagrama Visual CSS -->
      <div class="branch-visual q-my-md">
        <div class="line main">
          <span class="label">Main</span>
          <div class="dot"></div>
          <div class="dot"></div>
          <div class="dot"></div>
        </div>
        <div class="line feature">
          <div class="connector"></div>
          <span class="label">Feature</span>
          <div class="dot new"></div>
          <div class="dot new"></div>
        </div>
      </div>

      <TextBlock>
        Cuando estás en la rama <code>experimento-lidar</code>, cualquier archivo que crees o
        modifiques <strong>SOLO EXISTE AHÍ</strong>.
        <br />
        Si vuelves a <code>main</code> (`git switch main`), esos archivos desaparecerán mágicamente
        de tu carpeta. ¡No te asustes! Están seguros en la otra rama.
      </TextBlock>

      <CodeBlock
        title="Flujo de trabajo típico"
        lang="bash"
        content="# Estoy en main
git switch -c nueva-funcionalidad

# Hago cambios...
touch sensor_nuevo.py
git add .
git commit -m 'Añadir driver del sensor'

# Vuelvo a main
git switch main
ls
# ¡sensor_nuevo.py NO ESTÁ! (Magia)"
      />
    </div>

    <!-- 3. FUSIÓN (MERGE) -->
    <div class="section-group">
      <SectionTitle>3. Unificar Dimensiones (Merge)</SectionTitle>
      <TextBlock>
        El experimento fue un éxito. El código del sensor funciona. Ahora queremos traer esos
        cambios a la rama principal. Este proceso se llama <strong>Fusión (Merge)</strong>.
      </TextBlock>

      <AlertBlock type="warning" title="Regla de la Fusión">
        Siempre debes estar parado en la rama <strong>DESTINO</strong> (la que recibe los cambios).
        <br />
        "Estoy en main y quiero absorber a feature".
      </AlertBlock>

      <CodeBlock
        title="Fusionando"
        lang="bash"
        content="# 1. Ir a la rama principal
git switch main

# 2. Fusionar la rama experimental
git merge experimento-lidar

# 3. Borrar la rama experimental (ya no la necesitas)
git branch -d experimento-lidar"
        :copyable="true"
      />
    </div>

    <!-- 4. CONFLICTOS -->
    <div class="section-group">
      <SectionTitle>4. El Temido Conflicto</SectionTitle>
      <TextBlock>
        ¿Qué pasa si en <code>main</code> modificaste la línea 10 de un archivo, y en
        <code>feature</code>
        también modificaste la misma línea 10 pero con otro texto?
        <br /><br />
        Git entra en pánico y grita: <strong>CONFLICTO</strong>. No sabe cuál versión es la buena.
      </TextBlock>

      <CodeBlock
        title="Anatomía de un Conflicto"
        lang="python"
        content="<<<<<<< HEAD (main)
velocidad = 50
=======
velocidad = 100
>>>>>>> feature"
      />

      <StepsBlock
        :steps="[
          'Git detiene la fusión y marca el archivo con símbolos raros (<<<< ==== >>>>).',
          'Abre el archivo en tu editor (VS Code lo resalta bonito).',
          'Decide qué código se queda (o combina ambos). Borra los símbolos raros.',
          'Guarda el archivo.',
          'Ejecuta: git add archivo_arreglado.py',
          'Finaliza: git commit (sin mensaje, Git pondrá uno automático).',
        ]"
      />
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto del Multiverso</SectionTitle>
      <TextBlock>
        1. Crea un repo nuevo. 2. Crea un archivo <code>historia.txt</code> con el texto: "Había una
        vez un robot". Haz commit en main. 3. Crea una rama <code>villano</code>. 4. En esa rama,
        cambia el texto a: "Había una vez un robot malvado". Haz commit. 5. Vuelve a
        <code>main</code>. Verifica que el archivo dice "robot" (bueno). 6. Haz merge de
        <code>villano</code> en <code>main</code>. 7. Ahora tu historia oficial tiene un robot
        malvado.
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
import StepsBlock from 'components/content/StepsBlock.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3rem;
}

/* Visualización de Ramas */
.branch-visual {
  position: relative;
  height: 120px;
  background: rgba(15, 23, 42, 0.3);
  border-radius: 12px;
  padding: 20px;
  overflow: hidden;
}

.line {
  position: absolute;
  height: 4px;
  background: #475569;
  width: 80%;
  display: flex;
  align-items: center;
  border-radius: 2px;
}

.line.main {
  top: 30px;
  left: 10%;
  background: #3b82f6; /* Azul */
}

.line.feature {
  top: 80px;
  left: 30%;
  width: 60%;
  background: #10b981; /* Verde */
}

.connector {
  position: absolute;
  left: -2px;
  top: -50px;
  width: 4px;
  height: 54px;
  background: #10b981;
  transform: skewX(-20deg); /* Efecto de bifurcación */
  transform-origin: bottom;
}

.dot {
  width: 12px;
  height: 12px;
  border-radius: 50%;
  background: white;
  margin-left: 40px;
  box-shadow: 0 0 0 4px rgba(0, 0, 0, 0.3);
  z-index: 2;
}

.dot.new {
  background: #a7f3d0;
}

.label {
  position: absolute;
  left: -60px;
  font-family: 'Fira Code', monospace;
  font-size: 0.8rem;
  color: #cbd5e1;
  font-weight: bold;
}
</style>
