<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        No todo merece ser guardado. Cuando programas, generas archivos temporales, logs de errores,
        binarios compilados y carpetas de sistema que <strong>NO DEBEN</strong> estar en tu
        repositorio. <br /><br />
        El archivo <code>.gitignore</code> es el portero de la discoteca. Le das una lista de
        nombres indeseables y él se encarga de que Git los ignore para siempre, aunque hagas
        <code>git add .</code>.
      </TextBlock>
    </div>

    <!-- 1. ¿QUÉ IGNORAR? -->
    <div class="section-group">
      <SectionTitle>1. Los Indeseables</SectionTitle>
      <TextBlock>
        En el mundo de la robótica y Python, hay culpables habituales que ensucian los proyectos.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-4">
          <div class="ignore-card">
            <div class="text-subtitle1 text-accent">🐍 Python</div>
            <ul class="dense-list">
              <li><code>__pycache__/</code></li>
              <li><code>*.pyc</code></li>
              <li>Entornos virtuales (<code>venv/</code>)</li>
            </ul>
          </div>
        </div>
        <div class="col-12 col-md-4">
          <div class="ignore-card">
            <div class="text-subtitle1 text-primary">🤖 ROS 2</div>
            <ul class="dense-list">
              <li><code>build/</code> (Compilación)</li>
              <li><code>install/</code> (Binarios)</li>
              <li><code>log/</code> (Historiales)</li>
            </ul>
          </div>
        </div>
        <div class="col-12 col-md-4">
          <div class="ignore-card">
            <div class="text-subtitle1 text-secondary">💻 Sistema</div>
            <ul class="dense-list">
              <li><code>.DS_Store</code> (Mac)</li>
              <li><code>.vscode/</code> (Editor)</li>
              <li>Claves secretas (<code>.env</code>)</li>
            </ul>
          </div>
        </div>
      </div>
    </div>

    <!-- 2. CREANDO EL ARCHIVO -->
    <div class="section-group">
      <SectionTitle>2. Creando el Escudo (.gitignore)</SectionTitle>
      <TextBlock>
        El archivo debe llamarse exactamente <code>.gitignore</code> (con el punto al inicio) y
        estar en la raíz de tu proyecto.
      </TextBlock>

      <CodeBlock
        title="Contenido recomendado para ROS 2"
        lang="bash"
        content="# --- Python ---
__pycache__/
*.pyc

# --- ROS 2 Workspaces (CRÍTICO) ---
build/
install/
log/

# --- Editores ---
.vscode/
.idea/

# --- Sistema ---
.DS_Store"
        :copyable="true"
      />

      <AlertBlock type="info" title="Sintaxis">
        - <code>nombre/</code>: Ignora una carpeta entera. - <code>*.log</code>: Ignora cualquier
        archivo que termine en .log (Comodín). - <code>!importante.log</code>: El signo de
        exclamación es "Excepción" (No ignores este).
      </AlertBlock>
    </div>

    <!-- 3. EL ERROR COMÚN -->
    <div class="section-group">
      <SectionTitle>3. ¡Demasiado Tarde!</SectionTitle>
      <TextBlock>
        Un error muy común: Creas el <code>.gitignore</code> <strong>DESPUÉS</strong> de haber hecho
        commit de los archivos basura. <br /><br />
        Si Git ya está vigilando un archivo, el <code>.gitignore</code>
        <strong>NO LE AFECTA</strong>. Tienes que borrarlo de la memoria de Git primero.
      </TextBlock>

      <CodeBlock
        title="Limpiando el caché"
        lang="bash"
        content="# 1. Dile a Git que deje de vigilar la carpeta build (pero que NO la borre del disco)
git rm -r --cached build/

# 2. Ahora sí, el .gitignore funcionará
git status
# (Debería salir que se borró 'build/' del repo)

# 3. Guarda el cambio
git commit -m 'Dejar de trackear carpeta build'"
      />
    </div>

    <!-- 4. GENERADORES AUTOMÁTICOS -->
    <div class="section-group">
      <SectionTitle>4. No reinventes la rueda</SectionTitle>
      <TextBlock>
        No tienes que escribir estas listas de memoria. Existen sitios web como
        <strong>gitignore.io</strong>
        donde pones "Python", "ROS", "VisualStudioCode" y te generan el archivo perfecto.
      </TextBlock>

      <SplitBlock>
        <template #left>
          <TextBlock>
            En un proyecto profesional, el <code>.gitignore</code> es lo primero que se crea,
            incluso antes que el código.
          </TextBlock>
        </template>
        <template #right>
          <div class="text-center q-pa-md bg-dark rounded-borders">
            <div class="text-h2">🚫</div>
            <div class="text-caption text-grey">Si no está en gitignore, es público.</div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto de Limpieza</SectionTitle>
      <StepsBlock
        :steps="[
          'Crea una carpeta test_ignore.',
          'Inicia git init.',
          'Crea un archivo secreto.txt con una contraseña falsa.',
          'Verifica que git status lo ve (Rojo).',
          'Crea el archivo .gitignore y escribe dentro: secreto.txt',
          'Vuelve a hacer git status. ¡El archivo secreto ha desaparecido del radar!',
          'Añade y commitea el .gitignore.',
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

.ignore-card {
  background: var(--bg-surface-solid);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.2rem;
  height: 100%;
}

.dense-list {
  padding-left: 1.2rem;
  margin-top: 0.5rem;
  color: var(--text-secondary);
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
}
.dense-list li {
  margin-bottom: 4px;
}
</style>
