<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        Git no guarda archivos cada vez que parpadeas (como Google Docs). Git funciona por "paquetes
        de cambios" conscientes. Tú decides exactamente qué cambios guardar y cuándo.
        <br /><br />
        Para entender esto, imagina que eres un fotógrafo. No publicas todas las fotos que tomas.
        Primero tomas muchas (Working Directory), luego eliges las mejores para el álbum (Staging
        Area), y finalmente las imprimes y archivas (Commit).
      </TextBlock>
    </div>

    <!-- 1. LOS TRES ESTADOS -->
    <div class="section-group">
      <SectionTitle>1. Los Tres Estados de la Materia</SectionTitle>

      <!-- Diagrama Conceptual con CSS -->
      <div class="states-container q-my-lg">
        <div class="state-box working">
          <div class="state-icon">📝</div>
          <div class="state-title">Working Directory</div>
          <p>Tu carpeta actual. Aquí editas, borras y ensucias.</p>
          <div class="state-status text-red">Untracked / Modified</div>
        </div>

        <div class="arrow">➜ <br /><span class="text-caption">git add</span></div>

        <div class="state-box staging">
          <div class="state-icon">📦</div>
          <div class="state-title">Staging Area</div>
          <p>La "Zona de Preparación". Aquí apilas lo que vas a guardar.</p>
          <div class="state-status text-green">Staged</div>
        </div>

        <div class="arrow">➜ <br /><span class="text-caption">git commit</span></div>

        <div class="state-box repo">
          <div class="state-icon">🏛️</div>
          <div class="state-title">Repository (.git)</div>
          <p>La historia inmutable. Aquí viven las versiones guardadas.</p>
          <div class="state-status text-grey">Committed</div>
        </div>
      </div>
    </div>

    <!-- 2. INICIAR (INIT) -->
    <div class="section-group">
      <SectionTitle>2. El Big Bang (git init)</SectionTitle>
      <TextBlock>
        Para que una carpeta normal se convierta en un repositorio con superpoderes, usamos
        <code>git init</code>. Esto crea una carpeta oculta <code>.git</code> donde vive toda la
        magia.
      </TextBlock>

      <CodeBlock
        title="Creando el Universo"
        lang="bash"
        content="mkdir mi_robot_v1
cd mi_robot_v1

# Inicializar Git
git init
# Salida: Initialized empty Git repository in .../.git/

# Verificar estado (está vacío)
git status"
      />
    </div>

    <!-- 3. PREPARAR (ADD) -->
    <div class="section-group">
      <SectionTitle>3. Preparando la Foto (git add)</SectionTitle>
      <TextBlock>
        Creas un archivo. Git lo ve, pero no lo vigila (Untracked). Tienes que decirle
        explícitamente: "Oye, vigila esto".
      </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="text-subtitle2 text-grey-4">Paso A: Crear archivo</div>
          <CodeBlock
            lang="bash"
            content="touch main.py
git status
# Sale en ROJO (Untracked)"
          />
        </template>
        <template #right>
          <div class="text-subtitle2 text-accent">Paso B: Pasar al Staging</div>
          <CodeBlock
            lang="bash"
            content="git add main.py
git status
# Sale en VERDE (Changes to be committed)"
          />
        </template>
      </SplitBlock>

      <AlertBlock type="info" title="Truco: Añadir todo">
        Si modificaste 20 archivos, no los añadas uno por uno. Usa <code>git add .</code> (el punto
        significa "todo aquí").
      </AlertBlock>
    </div>

    <!-- 4. GUARDAR (COMMIT) -->
    <div class="section-group">
      <SectionTitle>4. Disparar la Cámara (git commit)</SectionTitle>
      <TextBlock>
        El <code>commit</code> toma todo lo que está en el Staging Area y lo congela en el tiempo
        con un mensaje descriptivo. Cada commit crea un ID único (Hash), como <code>a1b2c3d</code>.
      </TextBlock>

      <CodeBlock
        title="Creando historia"
        lang="bash"
        content="# La bandera -m es para el mensaje
git commit -m 'Crear estructura inicial del robot'

# Salida esperada:
# [main (root-commit) a1b2c3d] Crear estructura inicial del robot
# 1 file changed, 0 insertions(+), 0 deletions(-)"
        :copyable="true"
      />

      <div class="q-mt-md">
        <AlertBlock type="danger" title="Malos Mensajes vs Buenos Mensajes">
          ❌ <code>git commit -m "arreglos"</code> (Nadie sabe qué arreglaste)
          <br />
          ✅ <code>git commit -m "Corregir velocidad del motor y añadir sensor LIDAR"</code>
        </AlertBlock>
      </div>
    </div>

    <!-- 5. VER LA HISTORIA (LOG) -->
    <div class="section-group">
      <SectionTitle>5. Viajar en el Tiempo (git log)</SectionTitle>
      <TextBlock>
        ¿De qué sirve guardar si no puedes ver el pasado? El comando <code>git log</code> muestra el
        diario de tu proyecto.
      </TextBlock>

      <CodeBlock
        title="Consultando la bitácora"
        lang="bash"
        content="# Ver historial detallado
git log

# Ver historial resumido (ID + Mensaje)
git log --oneline

# Ver historial gráfico (si configuraste el alias 'lg' en la lección pasada)
git lg"
      />
    </div>

    <!-- 6. CICLO DE VIDA COMPLETO -->
    <div class="section-group">
      <SectionTitle>6. El Ciclo de Vida Real</SectionTitle>
      <TextBlock>
        Vamos a simular un día de trabajo normal. Modificamos un archivo existente y vemos cómo
        cambia de estado.
      </TextBlock>

      <StepsBlock
        :steps="[
          'Modificas main.py: echo \'print(\'Hola\')\' > main.py',
          'Estado: MODIFIED (Rojo). Git sabe que cambió, pero no lo ha preparado.',
          'Preparas: git add main.py',
          'Estado: STAGED (Verde). Listo para la foto.',
          'Guardas: git commit -m \'Añadir saludo inicial\'',
          'Estado: CLEAN. El Working Directory está limpio, igual que el Repo.',
        ]"
      />
    </div>

    <!-- 7. DESHACER (RESTORE) -->
    <div class="section-group">
      <SectionTitle>7. Pánico: ¡Rompí todo!</SectionTitle>
      <TextBlock>
        A veces editas algo y el código deja de funcionar. Quieres volver a como estaba en el último
        commit (limpiar el desastre).
      </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="text-subtitle2 text-warning">Descartar cambios (Unstaged)</div>
          <p>
            Borra tus cambios actuales en el archivo y lo deja como estaba la última vez que
            guardaste.
          </p>
          <CodeBlock lang="bash" content="git restore main.py" />
        </template>
        <template #right>
          <div class="text-subtitle2 text-accent">Sacar del Staging (Unstage)</div>
          <p>
            Si hiciste <code>git add</code> por error pero no quieres borrar los cambios, solo
            sacarlos de la caja.
          </p>
          <CodeBlock lang="bash" content="git restore --staged main.py" />
        </template>
      </SplitBlock>
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto del Historiador</SectionTitle>
      <TextBlock>
        En tu terminal: 1. Crea una carpeta <code>proyecto_prueba</code>. 2. Inicializa git. 3. Crea
        3 archivos de texto (capitulo1.txt, capitulo2.txt, notas.txt). 4. Añade y haz commit SOLO de
        los capítulos ("Primer borrador del libro"). 5. Añade las notas y haz otro commit ("Añadir
        notas personales"). 6. Ejecuta <code>git log --oneline</code>. Deberías ver 2 commits
        distintos.
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

/* Estilos del Diagrama de Estados */
.states-container {
  display: flex;
  justify-content: space-between;
  align-items: center;
  background: rgba(15, 23, 42, 0.4);
  padding: 20px;
  border-radius: 16px;
  overflow-x: auto;
}

.state-box {
  background: rgba(30, 41, 59, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 12px;
  padding: 16px;
  width: 220px;
  text-align: center;
  min-height: 180px;
  display: flex;
  flex-direction: column;
  align-items: center;
}

.state-icon {
  font-size: 2.5rem;
  margin-bottom: 8px;
}

.state-title {
  font-weight: bold;
  font-size: 1.1rem;
  margin-bottom: 8px;
  color: #f8fafc;
}

.state-box p {
  font-size: 0.85rem;
  color: #94a3b8;
  line-height: 1.4;
  flex-grow: 1;
}

.state-status {
  font-family: 'Fira Code', monospace;
  font-size: 0.8rem;
  font-weight: bold;
  padding: 4px 8px;
  border-radius: 4px;
  background: rgba(0, 0, 0, 0.2);
  margin-top: 8px;
}

.arrow {
  font-size: 1.5rem;
  color: #64748b;
  text-align: center;
  margin: 0 10px;
}

.text-red {
  color: #f87171;
}
.text-green {
  color: #4ade80;
}
.text-grey {
  color: #cbd5e1;
}
</style>
