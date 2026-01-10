<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        Git es un sistema de control de versiones distribuido. Piensa en él como un "punto de
        guardado" en un videojuego. Si te enfrentas a un jefe (un bug difícil) y mueres (rompes el
        código), puedes recargar la partida desde el último punto seguro.
        <br /><br />
        Pero antes de empezar, debemos configurar tu <strong>Identidad</strong>. Git firma cada
        línea de código que escribes con tu nombre y correo.
      </TextBlock>
    </div>

    <!-- 1. INSTALACIÓN -->
    <div class="section-group">
      <SectionTitle>1. Instalación y Verificación</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            En Ubuntu, Git suele venir instalado, pero es mejor asegurarse de tener la última
            versión estable.
            <br /><br />
            El comando <code>--version</code> es tu primer chequeo de salud.
          </TextBlock>
        </template>
        <template #right>
          <CodeBlock
            title="Terminal"
            lang="bash"
            content="# Instalar Git
sudo apt update
sudo apt install git

# Verificar versión
git --version
# Debería salir algo como: git version 2.34.1"
            :copyable="true"
          />
        </template>
      </SplitBlock>
    </div>

    <!-- 2. IDENTIDAD (CRÍTICO) -->
    <div class="section-group">
      <SectionTitle>2. Tu Identidad Digital (Global)</SectionTitle>

      <AlertBlock type="danger" title="⚠️ Importante para GitHub">
        El correo que pongas aquí <strong>DEBE</strong> coincidir con el que usaste para registrarte
        en GitHub.
        <br />
        Si no coinciden, tus contribuciones no aparecerán en tu perfil (los cuadritos verdes) y
        parecerás un fantasma.
      </AlertBlock>

      <TextBlock>
        Usamos la bandera <code>--global</code> para que esta configuración se aplique a TODOS tus
        proyectos en esta computadora.
      </TextBlock>

      <CodeBlock
        title="Configurando Usuario"
        lang="bash"
        content="# 1. Tu nombre (Aparecerá en el historial de cambios)
git config --global user.name 'Alexander Calderon'

# 2. Tu correo (El mismo de GitHub)
git config --global user.email 'alexander@ejemplo.com'"
        :copyable="true"
      />
    </div>

    <!-- 3. GLOBAL VS LOCAL -->
    <div class="section-group">
      <SectionTitle>3. Escenario: ¿Global o Local?</SectionTitle>
      <TextBlock>
        A veces trabajas en proyectos personales y proyectos de trabajo en la misma PC. No quieres
        usar tu correo personal para el código de la empresa.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="config-card global">
            <div class="text-h6 text-white">Configuración Global</div>
            <p>Es la "Licencia de Conducir". Te identifica por defecto en cualquier lugar.</p>
            <code>git config --global user.email "yo@gmail.com"</code>
            <div class="text-caption q-mt-sm text-grey-4">
              Se guarda en: <code>~/.gitconfig</code>
            </div>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="config-card local">
            <div class="text-h6 text-white">Configuración Local</div>
            <p>Es el "Gafete de la Oficina". Solo vale dentro de esa carpeta específica.</p>
            <code>git config --local user.email "ing@empresa.com"</code>
            <div class="text-caption q-mt-sm text-grey-4">
              Se guarda en: <code>.git/config</code> (dentro del proyecto)
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 4. EL EDITOR POR DEFECTO -->
    <div class="section-group">
      <SectionTitle>4. Evitando la "Trampa de Vim"</SectionTitle>
      <TextBlock>
        Cuando haces un commit sin mensaje, Git abre un editor de texto automáticamente. Por defecto
        suele ser <strong>Vim</strong>.
        <br />
        Para un principiante, Vim es una pesadilla (no puedes salir ni escribir fácilmente). Vamos a
        cambiarlo a
        <strong>Nano</strong> (simple) o <strong>VS Code</strong>.
      </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="text-weight-bold q-mb-sm">Opción A: Nano (Recomendado Terminal)</div>
          <CodeBlock lang="bash" content="git config --global core.editor nano" />
        </template>
        <template #right>
          <div class="text-weight-bold q-mb-sm">Opción B: VS Code (Recomendado Visual)</div>
          <CodeBlock lang="bash" content="git config --global core.editor 'code --wait'" />
        </template>
      </SplitBlock>
    </div>

    <!-- 5. RAMA PRINCIPAL -->
    <div class="section-group">
      <SectionTitle>5. Estandarización: Main vs Master</SectionTitle>
      <TextBlock>
        Antiguamente, la rama principal se llamaba <code>master</code>. Hoy en día, el estándar de
        la industria (y de GitHub) es llamarla <code>main</code>. <br /><br />
        Vamos a configurar Git para que cada vez que inicies un proyecto nuevo, use
        <code>main</code> por defecto.
      </TextBlock>

      <CodeBlock lang="bash" content="git config --global init.defaultBranch main" />
    </div>

    <!-- 6. ALIAS (PRODUCTIVIDAD) -->
    <div class="section-group">
      <SectionTitle>6. Alias: Trucos de Ingeniero</SectionTitle>
      <TextBlock>
        Los ingenieros somos vagos (eficientes). ¿Por qué escribir <code>git status</code> cien
        veces al día cuando puedes escribir <code>git st</code>?
        <br />
        Los alias son atajos personalizados.
      </TextBlock>

      <CodeBlock
        title="Creando Superpoderes"
        lang="bash"
        content="# Alias para status
git config --global alias.st status

# Alias para ver el historial como un árbol bonito (CRÍTICO)
git config --global alias.lg 'log --oneline --graph --decorate --all'

# Prueba tus nuevos comandos:
# git st
# git lg"
        :copyable="true"
      />
    </div>

    <!-- 7. VERIFICACIÓN FINAL -->
    <div class="section-group">
      <SectionTitle>7. Revisar tu Configuración</SectionTitle>
      <TextBlock>
        Para ver toda tu configuración actual y dónde está guardada cada variable, usa el comando
        list.
      </TextBlock>

      <CodeBlock title="Auditoría" lang="bash" content="git config --list --show-origin" />

      <AlertBlock type="info" title="Salida Esperada">
        file:/home/alexander/.gitconfig user.name=Alexander Calderon
        <br />
        file:/home/alexander/.gitconfig user.email=alexander@ejemplo.com
        <br />
        file:/home/alexander/.gitconfig init.defaultbranch=main
      </AlertBlock>
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto de Configuración</SectionTitle>
      <StepsBlock
        :steps="[
          'Abre tu terminal.',
          'Configura tu user.name y user.email reales.',
          'Configura el editor Nano como predeterminado.',
          'Crea el alias \'git lg\' (es el comando más útil que tendrás).',
          'Ejecuta \'git config --list\' y toma una captura mental de tu identidad.',
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

.config-card {
  padding: 1.5rem;
  border-radius: 12px;
  height: 100%;
  border: 1px solid rgba(255, 255, 255, 0.1);
  display: flex;
  flex-direction: column;
  justify-content: center;
}

.config-card.global {
  background: linear-gradient(135deg, rgba(59, 130, 246, 0.2), rgba(30, 41, 59, 0.8));
  border-left: 4px solid #3b82f6; /* Azul */
}

.config-card.local {
  background: linear-gradient(135deg, rgba(16, 185, 129, 0.2), rgba(30, 41, 59, 0.8));
  border-left: 4px solid #10b981; /* Verde */
}

.config-card p {
  color: #cbd5e1;
  margin: 10px 0;
}

.config-card code {
  background: rgba(0, 0, 0, 0.3);
  padding: 8px;
  border-radius: 4px;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
  color: #fbbf24;
  word-break: break-all;
}
</style>
