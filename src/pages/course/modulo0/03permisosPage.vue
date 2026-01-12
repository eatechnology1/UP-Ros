<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      Linux no es una democracia; es una meritocracia militarizada. Cada archivo tiene un guardia
      que verifica tu credencial antes de dejarte pasar. Aprende quién tiene la llave nuclear (Root)
      y cómo autorizar a tu robot para acceder al hardware sin comprometer la seguridad.
    </TextBlock>

    <AlertBlock type="info" title="¿Por qué importa en Robótica?">
      Tu robot necesita acceso a puertos USB (lidar, cámara), archivos de configuración, y ejecutar
      scripts. Sin permisos correctos, tu código compilará pero no correrá. Dominar esto te ahorrará
      horas de debugging.
    </AlertBlock>

    <!-- SISTEMA DE PERMISOS -->
    <div class="section-group">
      <SectionTitle>1. El Sistema de Tres Niveles</SectionTitle>
      <TextBlock>
        Cada archivo/carpeta en Linux tiene permisos para <strong>tres categorías</strong> de
        usuarios:
      </TextBlock>

      <div class="levels-grid q-mt-md">
        <div class="level-card owner">
          <q-icon name="person" size="xl" />
          <div class="level-title">Propietario (User)</div>
          <div class="level-desc">El creador del archivo. Tú en tu /home</div>
        </div>
        <div class="level-card group">
          <q-icon name="group" size="xl" />
          <div class="level-title">Grupo (Group)</div>
          <div class="level-desc">Equipo con acceso compartido (ej: dialout)</div>
        </div>
        <div class="level-card others">
          <q-icon name="public" size="xl" />
          <div class="level-title">Otros (Others)</div>
          <div class="level-desc">El resto del mundo</div>
        </div>
      </div>

      <div class="q-mt-lg">
        <TextBlock> Cada nivel puede tener <strong>tres permisos</strong>: </TextBlock>
        <div class="permissions-grid q-mt-sm">
          <div class="perm-card read">
            <div class="perm-symbol">r</div>
            <div class="perm-name">Read (Leer)</div>
            <div class="perm-value">Valor: 4</div>
          </div>
          <div class="perm-card write">
            <div class="perm-symbol">w</div>
            <div class="perm-name">Write (Escribir)</div>
            <div class="perm-value">Valor: 2</div>
          </div>
          <div class="perm-card execute">
            <div class="perm-symbol">x</div>
            <div class="perm-name">Execute (Ejecutar)</div>
            <div class="perm-value">Valor: 1</div>
          </div>
        </div>
      </div>
    </div>

    <!-- DECODIFICAR LS -L -->
    <div class="section-group">
      <SectionTitle>2. Decodificando ls -l</SectionTitle>
      <CodeBlock
        :hide-header="true"
        lang="bash"
        content="-rwxr-xr-- 1 alexander dialout 2048 Jan 10 14:30 robot.py"
      />

      <div class="decoder-grid q-mt-md">
        <div class="decoder-item">
          <div class="decoder-label">-</div>
          <div class="decoder-desc">Tipo: - (archivo), d (directorio), l (link)</div>
        </div>
        <div class="decoder-item highlight">
          <div class="decoder-label">rwx</div>
          <div class="decoder-desc">Propietario: Lectura + Escritura + Ejecución</div>
        </div>
        <div class="decoder-item highlight">
          <div class="decoder-label">r-x</div>
          <div class="decoder-desc">Grupo: Lectura + Ejecución (sin escritura)</div>
        </div>
        <div class="decoder-item highlight">
          <div class="decoder-label">r--</div>
          <div class="decoder-desc">Otros: Solo lectura</div>
        </div>
        <div class="decoder-item">
          <div class="decoder-label">alexander</div>
          <div class="decoder-desc">Propietario del archivo</div>
        </div>
        <div class="decoder-item">
          <div class="decoder-label">dialout</div>
          <div class="decoder-desc">Grupo asignado</div>
        </div>
      </div>
    </div>

    <!-- CHMOD -->
    <div class="section-group">
      <SectionTitle>3. Cambiar Permisos: chmod</SectionTitle>

      <div class="row q-col-gutter-md">
        <div class="col-12 col-md-6">
          <div class="method-card">
            <div class="method-title">Método Numérico (Octal)</div>
            <TextBlock>
              Suma los valores: <code>r=4</code>, <code>w=2</code>, <code>x=1</code>
            </TextBlock>
            <div class="calc-examples">
              <div class="calc-row"><code>7</code> = rwx (4+2+1) = Todo</div>
              <div class="calc-row"><code>6</code> = rw- (4+2+0) = Leer y escribir</div>
              <div class="calc-row"><code>5</code> = r-x (4+0+1) = Leer y ejecutar</div>
              <div class="calc-row"><code>4</code> = r-- (4+0+0) = Solo leer</div>
            </div>
            <CodeBlock
              lang="bash"
              content="chmod 755 script.py
# 7(owner) 5(group) 5(others)"
              :copyable="true"
            />
          </div>
        </div>

        <div class="col-12 col-md-6">
          <div class="method-card">
            <div class="method-title">Método Simbólico</div>
            <TextBlock>
              Más legible: <code>u</code> (user), <code>g</code> (group), <code>o</code> (others),
              <code>a</code> (all)
            </TextBlock>
            <CodeBlock
              lang="bash"
              content="# Agregar ejecución al propietario
chmod u+x script.py

# Quitar escritura al grupo
chmod g-w config.yaml

# Dar lectura a todos
chmod a+r readme.txt"
              :copyable="true"
            />
          </div>
        </div>
      </div>

      <AlertBlock type="warning" title="⚠️ Nunca uses chmod 777" class="q-mt-md">
        <code>chmod 777</code> da permisos totales a TODO EL MUNDO. Es como dejar tu casa sin
        puertas. Usa <code>755</code> para scripts o <code>644</code> para archivos de datos.
      </AlertBlock>
    </div>

    <!-- CASOS DE USO -->
    <div class="section-group">
      <SectionTitle>4. Casos de Uso en ROS 2</SectionTitle>

      <div class="use-cases">
        <div class="use-case">
          <div class="use-case-header">
            <q-icon name="play_circle" color="green-4" size="md" />
            <span>Hacer un Script Ejecutable</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Crear script
touch mi_nodo.py

# Dar permisos de ejecución
chmod +x mi_nodo.py

# Ahora puedes ejecutarlo directamente
./mi_nodo.py"
            :copyable="true"
          />
        </div>

        <div class="use-case">
          <div class="use-case-header">
            <q-icon name="lock" color="orange-4" size="md" />
            <span>Proteger Configuraciones Críticas</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Solo lectura para evitar modificaciones accidentales
chmod 444 nav2_params.yaml

# Para editarlo, primero restaura permisos
chmod 644 nav2_params.yaml"
            :copyable="true"
          />
        </div>

        <div class="use-case">
          <div class="use-case-header">
            <q-icon name="folder" color="blue-4" size="md" />
            <span>Permisos de Directorio</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Recursivo: aplicar a carpeta y todo su contenido
chmod -R 755 mi_paquete/

# x en directorios = permiso para entrar (cd)"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- SUDO -->
    <div class="section-group">
      <SectionTitle>5. SUDO: Superpoderes Temporales</SectionTitle>
      <TextBlock>
        <code>sudo</code> (SuperUser DO) ejecuta un comando con permisos de administrador (root).
        Úsalo solo cuando sea necesario.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-md">
        <div class="col-12 col-md-6">
          <AlertBlock type="success" title="✅ Usos Correctos">
            <ul class="q-pl-md">
              <li>Instalar software: <code>sudo apt install</code></li>
              <li>Editar archivos del sistema: <code>sudo nano /etc/hosts</code></li>
              <li>Reiniciar servicios: <code>sudo systemctl restart</code></li>
            </ul>
          </AlertBlock>
        </div>
        <div class="col-12 col-md-6">
          <AlertBlock type="danger" title="❌ Errores Fatales">
            <ul class="q-pl-md">
              <li><strong>NUNCA:</strong> <code>sudo colcon build</code></li>
              <li><strong>NUNCA:</strong> <code>sudo chmod 777 -R /</code></li>
              <li><strong>NUNCA:</strong> <code>sudo rm -rf /*</code></li>
            </ul>
          </AlertBlock>
        </div>
      </div>

      <AlertBlock type="warning" title="¿Por qué no sudo colcon build?" class="q-mt-md">
        Si compilas con <code>sudo</code>, los archivos en <code>build/</code> e
        <code>install/</code> pertenecerán a root. Tu usuario normal no podrá modificarlos después,
        rompiendo tu workspace.
      </AlertBlock>
    </div>

    <!-- GRUPOS -->
    <div class="section-group">
      <SectionTitle>6. Grupos de Linux: Acceso a Hardware</SectionTitle>
      <TextBlock>
        Los dispositivos de hardware (USB, video, audio) pertenecen a grupos específicos. Para
        acceder a ellos, tu usuario debe estar en el grupo correcto.
      </TextBlock>

      <div class="groups-table q-mt-md">
        <div class="group-row header">
          <div>Grupo</div>
          <div>Acceso a</div>
          <div>Comando</div>
        </div>
        <div class="group-row">
          <div><code>dialout</code></div>
          <div>Puertos seriales (Arduino, Lidar, GPS)</div>
          <div><code>sudo usermod -aG dialout $USER</code></div>
        </div>
        <div class="group-row">
          <div><code>video</code></div>
          <div>Cámaras USB, webcams</div>
          <div><code>sudo usermod -aG video $USER</code></div>
        </div>
        <div class="group-row">
          <div><code>sudo</code></div>
          <div>Ejecutar comandos con sudo</div>
          <div><code>sudo usermod -aG sudo $USER</code></div>
        </div>
      </div>

      <AlertBlock type="info" title="⚠️ Reinicio Obligatorio" class="q-mt-md">
        Después de agregar tu usuario a un grupo, <strong>debes reiniciar el PC</strong> (o cerrar
        sesión) para que los cambios surtan efecto. Verifica con <code>groups</code>.
      </AlertBlock>
    </div>

    <!-- ERRORES COMUNES -->
    <div class="section-group">
      <SectionTitle>Errores Comunes y Soluciones</SectionTitle>

      <q-expansion-item
        icon="error"
        label="bash: ./script.py: Permission denied"
        header-class="error-header"
      >
        <div class="error-content">
          <strong>Causa:</strong> El archivo no tiene permisos de ejecución. <br /><br />
          <strong>Solución:</strong>
          <CodeBlock lang="bash" content="chmod +x script.py" :copyable="true" />
        </div>
      </q-expansion-item>

      <q-expansion-item
        icon="error"
        label="Error opening serial port /dev/ttyUSB0: Permission denied"
        header-class="error-header"
        class="q-mt-sm"
      >
        <div class="error-content">
          <strong>Causa:</strong> Tu usuario no está en el grupo <code>dialout</code>. <br /><br />
          <strong>Solución:</strong>
          <CodeBlock
            lang="bash"
            content="sudo usermod -aG dialout $USER
# Reinicia el PC y verifica con: groups"
            :copyable="true"
          />
        </div>
      </q-expansion-item>
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://www.youtube.com/embed/Romc22GgusU"
            title="Permisos en Linux"
            frameborder="0"
            allow="
              accelerometer;
              autoplay;
              clipboard-write;
              encrypted-media;
              gyroscope;
              picture-in-picture;
            "
            allowfullscreen
          ></iframe>
        </div>
        <div class="video-caption">
          <q-icon name="info" color="blue-4" size="sm" />
          Video En progreso
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>ls -l</code>
          <span>Ver permisos de archivos</span>
        </div>
        <div class="summary-item">
          <code>chmod 755</code>
          <span>Cambiar permisos (numérico)</span>
        </div>
        <div class="summary-item">
          <code>chmod +x</code>
          <span>Hacer ejecutable (simbólico)</span>
        </div>
        <div class="summary-item">
          <code>sudo comando</code>
          <span>Ejecutar como root</span>
        </div>
        <div class="summary-item">
          <code>usermod -aG</code>
          <span>Agregar usuario a grupo</span>
        </div>
        <div class="summary-item">
          <code>groups</code>
          <span>Ver tus grupos actuales</span>
        </div>
      </div>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3.5rem;
}

.levels-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
}

.level-card {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.9), rgba(30, 41, 59, 0.9));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  text-align: center;
}

.level-card.owner {
  border-top: 3px solid #22c55e;
}
.level-card.group {
  border-top: 3px solid #3b82f6;
}
.level-card.others {
  border-top: 3px solid #94a3b8;
}

.level-title {
  font-weight: 700;
  color: #f1f5f9;
  margin: 0.75rem 0 0.5rem;
}

.level-desc {
  font-size: 0.85rem;
  color: #cbd5e1;
}

.permissions-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
  gap: 1rem;
}

.perm-card {
  background: rgba(15, 23, 42, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 1rem;
  text-align: center;
}

.perm-symbol {
  font-family: 'Fira Code', monospace;
  font-size: 2rem;
  font-weight: 700;
  margin-bottom: 0.5rem;
}

.perm-card.read .perm-symbol {
  color: #22c55e;
}
.perm-card.write .perm-symbol {
  color: #f59e0b;
}
.perm-card.execute .perm-symbol {
  color: #ef4444;
}

.perm-name {
  font-size: 0.9rem;
  color: #e2e8f0;
  margin-bottom: 0.25rem;
}

.perm-value {
  font-size: 0.8rem;
  color: #94a3b8;
  font-family: 'Fira Code', monospace;
}

.decoder-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 0.75rem;
}

.decoder-item {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 0.75rem;
}

.decoder-item.highlight {
  border-color: #3b82f6;
  background: rgba(59, 130, 246, 0.1);
}

.decoder-label {
  font-family: 'Fira Code', monospace;
  font-size: 1.1rem;
  font-weight: 700;
  color: #60a5fa;
  margin-bottom: 0.25rem;
}

.decoder-desc {
  font-size: 0.85rem;
  color: #cbd5e1;
}

.method-card {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.9), rgba(30, 41, 59, 0.9));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  height: 100%;
}

.method-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1rem;
}

.calc-examples {
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
  padding: 1rem;
  margin: 1rem 0;
}

.calc-row {
  font-family: 'Fira Code', monospace;
  color: #cbd5e1;
  margin-bottom: 0.5rem;
}

.use-cases {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.use-case {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.use-case-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: rgba(59, 130, 246, 0.1);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: #f1f5f9;
}

.groups-table {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.group-row {
  display: grid;
  grid-template-columns: 1fr 2fr 2fr;
  gap: 1rem;
  padding: 1rem 1.5rem;
  border-bottom: 1px solid rgba(148, 163, 184, 0.1);
}

.group-row:last-child {
  border-bottom: none;
}

.group-row.header {
  background: rgba(59, 130, 246, 0.1);
  font-weight: 700;
  color: #60a5fa;
}

.group-row div {
  display: flex;
  align-items: center;
  color: #e2e8f0;
  font-size: 0.9rem;
}

:deep(.error-header) {
  background: rgba(239, 68, 68, 0.1);
  border: 1px solid rgba(239, 68, 68, 0.3);
  border-radius: 8px;
  color: #fca5a5;
}

.error-content {
  background: rgba(15, 23, 42, 0.6);
  padding: 1.5rem;
  color: #cbd5e1;
}

.video-container {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.9), rgba(30, 41, 59, 0.9));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 1.5rem;
}

.video-wrapper {
  position: relative;
  padding-bottom: 56.25%;
  height: 0;
  overflow: hidden;
  border-radius: 12px;
  background: #000;
}

.video-wrapper iframe {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
}

.video-caption {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-top: 1rem;
  padding: 0.75rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 8px;
  color: #94a3b8;
  font-size: 0.85rem;
}

.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 1rem;
}

.summary-item {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.summary-item code {
  font-family: 'Fira Code', monospace;
  color: #22c55e;
  font-size: 1rem;
}

.summary-item span {
  color: #cbd5e1;
  font-size: 0.85rem;
}

@media (max-width: 768px) {
  .group-row {
    grid-template-columns: 1fr;
    gap: 0.5rem;
  }
}
</style>
