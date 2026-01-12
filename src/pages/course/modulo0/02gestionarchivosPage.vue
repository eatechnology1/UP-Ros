<template>
  <LessonContainer>
    <!-- ============================================ -->
    <!-- SECCIÓN 1: CONTEXTO E IMPORTANCIA -->
    <!-- ============================================ -->
    <TextBlock>
      Ya sabes moverte por el sistema. Ahora aprenderás a
      <strong>manipular la materia digital</strong>: crear archivos, duplicarlos, moverlos y
      destruirlos. <br /><br />
      En robótica, esto es el día a día: organizar logs de sensores, hacer backups de
      configuraciones, limpiar archivos temporales de simulaciones, y estructurar tu workspace de
      ROS 2. <strong>Un sistema desorganizado es un robot que falla.</strong>
    </TextBlock>

    <AlertBlock type="info" title="La Filosofía Unix: Todo es un Archivo">
      En Linux, <strong>todo</strong> es un archivo. Tu código Python es un archivo. La
      configuración de tu robot es un archivo. Incluso los dispositivos de hardware (cámara, lidar)
      aparecen como archivos en <code>/dev</code>. <br /><br />
      Dominar la gestión de archivos es dominar el sistema operativo.
    </AlertBlock>

    <!-- ============================================ -->
    <!-- SECCIÓN 2: MKDIR - CREAR DIRECTORIOS -->
    <!-- ============================================ -->
    <div class="section-group">
      <SectionTitle>1. Crear Directorios: mkdir (Make Directory)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Antes de crear archivos, necesitas un lugar donde guardarlos. El comando
            <code>mkdir</code> crea carpetas (directorios). <br /><br />
            <strong>Regla de oro:</strong> Organiza tu código en carpetas lógicas desde el día 1. Un
            proyecto robótico puede tener cientos de archivos.
          </TextBlock>

          <CodeBlock
            title="Crear una carpeta simple"
            lang="bash"
            content="mkdir mi_robot
ls -l"
            :copyable="true"
          />

          <div class="q-mt-md">
            <AlertBlock type="success" title="Bandera -p: Crear Jerarquías">
              La bandera <code>-p</code> (parents) crea toda la estructura de carpetas de una vez,
              incluso si las carpetas intermedias no existen.
            </AlertBlock>
          </div>
        </template>

        <template #right>
          <div class="hierarchy-demo">
            <div class="demo-title">Estructura de Workspace ROS 2</div>
            <CodeBlock
              :hide-header="true"
              lang="bash"
              content="mkdir -p ros2_ws/src/mi_robot/launch
mkdir -p ros2_ws/src/mi_robot/config
mkdir -p ros2_ws/src/mi_robot/scripts

# Resultado:
# ros2_ws/
# └── src/
#     └── mi_robot/
#         ├── launch/
#         ├── config/
#         └── scripts/"
            />
            <div class="demo-note">
              <q-icon name="lightbulb" color="yellow-6" size="sm" />
              <span>Sin <code>-p</code>, tendrías que crear cada carpeta una por una.</span>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- ============================================ -->
    <!-- SECCIÓN 3: TOUCH - CREAR ARCHIVOS -->
    <!-- ============================================ -->
    <div class="section-group">
      <SectionTitle>2. Crear Archivos Vacíos: touch</SectionTitle>
      <div class="row q-col-gutter-md">
        <div class="col-12 col-md-6">
          <div class="concept-card">
            <div class="concept-header">
              <q-icon name="note_add" color="green-4" size="lg" />
              <div class="concept-title">Uso Principal</div>
            </div>
            <div class="concept-body">
              <TextBlock>
                <code>touch</code> crea un archivo totalmente vacío (0 bytes). Es como sacar una
                hoja de papel en blanco. <br /><br />
                <strong>¿Por qué crear archivos vacíos?</strong> Para reservar el nombre antes de
                escribir código, o para crear placeholders en tu estructura de proyecto.
              </TextBlock>
              <CodeBlock
                lang="bash"
                content="touch nodo_control.py
touch config.yaml
ls -lh"
                :copyable="true"
              />
            </div>
          </div>
        </div>

        <div class="col-12 col-md-6">
          <div class="concept-card">
            <div class="concept-header">
              <q-icon name="schedule" color="blue-4" size="lg" />
              <div class="concept-title">Uso Secundario</div>
            </div>
            <div class="concept-body">
              <TextBlock>
                Si el archivo <strong>ya existe</strong>, <code>touch</code> solo actualiza su
                "timestamp" (fecha de modificación) sin borrar el contenido. <br /><br />
                Esto es útil para "marcar" archivos como recién modificados en sistemas de
                compilación.
              </TextBlock>
              <CodeBlock
                lang="bash"
                content="# Archivo existe con contenido
touch archivo_viejo.txt

# Solo cambia la fecha, contenido intacto
ls -l archivo_viejo.txt"
                :copyable="true"
              />
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-md">
        <AlertBlock type="info" title="Crear Múltiples Archivos a la Vez">
          Puedes crear varios archivos en un solo comando separándolos con espacios:
          <br />
          <code>touch archivo1.txt archivo2.txt archivo3.txt</code>
        </AlertBlock>
      </div>
    </div>

    <!-- ============================================ -->
    <!-- SECCIÓN 4: CP - COPIAR -->
    <!-- ============================================ -->
    <div class="section-group">
      <SectionTitle>3. Copiar Archivos y Directorios: cp (Copy)</SectionTitle>
      <TextBlock>
        El comando <code>cp</code> duplica archivos. Necesita dos argumentos:
        <strong>Origen</strong> (qué copiar) y <strong>Destino</strong> (dónde copiarlo).
      </TextBlock>

      <div class="syntax-card q-mt-md">
        <div class="syntax-label">Sintaxis General</div>
        <div class="syntax-pattern">
          <span class="syntax-cmd">cp</span>
          <span class="syntax-arg">[opciones]</span>
          <span class="syntax-arg origin">origen</span>
          <span class="syntax-arg dest">destino</span>
        </div>
      </div>

      <div class="row q-col-gutter-md q-mt-lg">
        <div class="col-12 col-md-4">
          <div class="operation-card">
            <div class="operation-icon">
              <q-icon name="content_copy" color="cyan-4" size="xl" />
            </div>
            <div class="operation-title">Copiar Archivo</div>
            <div class="operation-desc">Duplica un archivo en el mismo directorio o en otro</div>
            <CodeBlock
              :hide-header="true"
              lang="bash"
              content="# Mismo directorio
cp robot.py robot_backup.py

# Otro directorio
cp robot.py ~/backups/"
            />
            <div class="operation-note">El original permanece intacto</div>
          </div>
        </div>

        <div class="col-12 col-md-4">
          <div class="operation-card">
            <div class="operation-icon">
              <q-icon name="folder_copy" color="purple-4" size="xl" />
            </div>
            <div class="operation-title">Copiar Directorio</div>
            <div class="operation-desc">
              Requiere la bandera <code>-r</code> (recursive) para copiar todo el contenido
            </div>
            <CodeBlock
              :hide-header="true"
              lang="bash"
              content="# Copiar carpeta completa
cp -r mi_robot/ mi_robot_backup/

# Copia TODO: archivos y subcarpetas"
            />
            <div class="operation-note">Sin <code>-r</code>, obtendrás un error</div>
          </div>
        </div>

        <div class="col-12 col-md-4">
          <div class="operation-card">
            <div class="operation-icon">
              <q-icon name="update" color="orange-4" size="xl" />
            </div>
            <div class="operation-title">Copiar con Confirmación</div>
            <div class="operation-desc">
              La bandera <code>-i</code> (interactive) pregunta antes de sobrescribir
            </div>
            <CodeBlock
              :hide-header="true"
              lang="bash"
              content="# Pregunta si el destino existe
cp -i robot.py robot_backup.py

# Output: overwrite 'robot_backup.py'? (y/n)"
            />
            <div class="operation-note">Evita pérdidas accidentales de datos</div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="warning" title="Caso de Uso Real: Backup de Configuraciones">
          Antes de modificar un archivo crítico (como un launch file o YAML de Nav2), siempre haz
          una copia de seguridad:
          <br /><br />
          <code>cp nav2_params.yaml nav2_params.yaml.backup</code>
          <br /><br />
          Si algo sale mal, puedes restaurar el original con
          <code>cp nav2_params.yaml.backup nav2_params.yaml</code>
        </AlertBlock>
      </div>
    </div>

    <!-- ============================================ -->
    <!-- SECCIÓN 5: MV - MOVER Y RENOMBRAR -->
    <!-- ============================================ -->
    <div class="section-group">
      <SectionTitle>4. Mover y Renombrar: mv (Move)</SectionTitle>
      <TextBlock>
        En Linux, <strong>Mover</strong> y <strong>Renombrar</strong> son la misma operación:
        <code>mv</code>. La diferencia está en el destino que especifiques.
      </TextBlock>

      <div class="mv-comparison q-mt-md">
        <div class="mv-case">
          <div class="mv-case-header case-move">
            <q-icon name="drive_file_move" size="md" />
            <span>Caso A: Mover a Otra Carpeta</span>
          </div>
          <div class="mv-case-body">
            <TextBlock>
              Si el destino es una <strong>carpeta existente</strong>, el archivo se mueve dentro de
              ella manteniendo su nombre.
            </TextBlock>
            <CodeBlock
              lang="bash"
              content="# Estructura inicial:
# .
# ├── robot.py
# └── scripts/

mv robot.py scripts/

# Resultado:
# .
# └── scripts/
#     └── robot.py"
              :copyable="true"
            />
          </div>
        </div>

        <div class="mv-case">
          <div class="mv-case-header case-rename">
            <q-icon name="edit" size="md" />
            <span>Caso B: Renombrar en el Mismo Lugar</span>
          </div>
          <div class="mv-case-body">
            <TextBlock>
              Si el destino es un <strong>nombre nuevo</strong> (no una carpeta existente), el
              archivo cambia de nombre en el mismo directorio.
            </TextBlock>
            <CodeBlock
              lang="bash"
              content="# Renombrar archivo
mv robot_viejo.py robot_nuevo.py

# Renombrar carpeta
mv carpeta_vieja/ carpeta_nueva/"
              :copyable="true"
            />
          </div>
        </div>

        <div class="mv-case">
          <div class="mv-case-header case-both">
            <q-icon name="compare_arrows" size="md" />
            <span>Caso C: Mover Y Renombrar</span>
          </div>
          <div class="mv-case-body">
            <TextBlock>
              Puedes hacer ambas cosas a la vez: mover a otra carpeta y cambiar el nombre.
            </TextBlock>
            <CodeBlock
              lang="bash"
              content="# Mover y renombrar simultáneamente
mv robot.py scripts/control_node.py

# El archivo cambia de nombre Y de ubicación"
              :copyable="true"
            />
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="success" title="Tip Pro: Renombrar Múltiples Archivos">
          Para renombrar muchos archivos a la vez, usa un loop en bash:
          <br /><br />
          <code>for f in *.txt; do mv "$f" "${f%.txt}.log"; done</code>
          <br /><br />
          Esto convierte todos los <code>.txt</code> en <code>.log</code>
        </AlertBlock>
      </div>
    </div>

    <!-- ============================================ -->
    <!-- SECCIÓN 6: RM - ELIMINAR (PELIGRO) -->
    <!-- ============================================ -->
    <div class="section-group">
      <SectionTitle>5. Eliminar Archivos y Directorios: rm (Remove)</SectionTitle>

      <div class="danger-zone">
        <div class="danger-header">
          <q-icon name="warning" size="xl" color="red-5" />
          <div class="danger-title">⚠️ ZONA DE PELIGRO ⚠️</div>
        </div>
        <div class="danger-body">
          <TextBlock>
            En la terminal <strong>NO EXISTE LA PAPELERA DE RECICLAJE</strong>.
            <br />
            Cuando borras algo con <code>rm</code>, desaparece para siempre. No hay "Ctrl+Z". No hay
            "Restaurar". <br /><br />
            <strong>Regla de supervivencia:</strong> Siempre verifica dos veces antes de presionar
            Enter en un comando <code>rm</code>.
          </TextBlock>
        </div>
      </div>

      <div class="row q-col-gutter-md q-mt-lg">
        <div class="col-12 col-md-4">
          <div class="rm-card rm-file">
            <div class="rm-header">
              <q-icon name="description" size="lg" />
              <div class="rm-title">Borrar Archivo</div>
            </div>
            <CodeBlock :hide-header="true" lang="bash" content="rm archivo.txt" />
            <div class="rm-note">Simple y directo. El archivo desaparece.</div>
          </div>
        </div>

        <div class="col-12 col-md-4">
          <div class="rm-card rm-interactive">
            <div class="rm-header">
              <q-icon name="help" size="lg" />
              <div class="rm-title">Borrar con Confirmación</div>
            </div>
            <CodeBlock :hide-header="true" lang="bash" content="rm -i archivo.txt" />
            <div class="rm-note">
              Pregunta antes de borrar: <code>remove archivo.txt? (y/n)</code>
            </div>
          </div>
        </div>

        <div class="col-12 col-md-4">
          <div class="rm-card rm-recursive">
            <div class="rm-header">
              <q-icon name="folder_delete" size="lg" />
              <div class="rm-title">Borrar Directorio</div>
            </div>
            <CodeBlock :hide-header="true" lang="bash" content="rm -r carpeta/" />
            <div class="rm-note">
              <code>-r</code> (recursive) borra la carpeta y todo su contenido
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="danger" title="El Comando Más Peligroso del Universo">
          <code>rm -rf /</code> (con permisos de root) borraría <strong>TODO</strong> tu sistema
          operativo. <br /><br />
          <strong>Nunca</strong> copies comandos de internet sin entenderlos. Un espacio mal puesto
          puede destruir tu trabajo de meses. <br /><br />
          Ejemplo peligroso: <code>rm -rf carpeta /</code> (nota el espacio antes de <code>/</code>)
          borraría la carpeta Y la raíz del sistema.
        </AlertBlock>
      </div>

      <div class="q-mt-md">
        <SplitBlock>
          <template #left>
            <AlertBlock type="info" title="Alternativa Segura: Mover a Papelera">
              Algunos sistemas tienen el comando <code>trash</code> o <code>gio trash</code> que
              mueve archivos a la papelera en lugar de borrarlos permanentemente. <br /><br />
              <code>gio trash archivo.txt</code>
            </AlertBlock>
          </template>

          <template #right>
            <AlertBlock type="success" title="Protección: Alias Seguro">
              Agrega esto a tu <code>~/.bashrc</code> para que <code>rm</code> siempre pregunte:
              <br /><br />
              <code>alias rm='rm -i'</code>
              <br /><br />
              Después de editar, ejecuta <code>source ~/.bashrc</code>
            </AlertBlock>
          </template>
        </SplitBlock>
      </div>
    </div>

    <!-- ============================================ -->
    <!-- SECCIÓN 7: WILDCARDS (COMODINES) -->
    <!-- ============================================ -->
    <div class="section-group">
      <SectionTitle>6. Wildcards: Operaciones en Masa</SectionTitle>
      <TextBlock>
        Los <strong>wildcards</strong> (comodines) son caracteres especiales que representan
        múltiples archivos a la vez. Son esenciales para trabajar eficientemente.
      </TextBlock>

      <div class="wildcards-grid q-mt-md">
        <div class="wildcard-card">
          <div class="wildcard-symbol">*</div>
          <div class="wildcard-name">Asterisco</div>
          <div class="wildcard-desc">
            Representa <strong>cualquier cantidad</strong> de caracteres
          </div>
          <div class="wildcard-examples">
            <div class="example-item"><code>*.py</code> → Todos los archivos Python</div>
            <div class="example-item">
              <code>robot_*</code> → robot_v1.py, robot_v2.py, robot_final.py
            </div>
            <div class="example-item"><code>*</code> → Absolutamente todos los archivos</div>
          </div>
        </div>

        <div class="wildcard-card">
          <div class="wildcard-symbol">?</div>
          <div class="wildcard-name">Signo de Interrogación</div>
          <div class="wildcard-desc">Representa <strong>exactamente un</strong> carácter</div>
          <div class="wildcard-examples">
            <div class="example-item"><code>log?.txt</code> → log1.txt, log2.txt, logA.txt</div>
            <div class="example-item">
              <code>????.py</code> → Archivos Python de exactamente 4 letras
            </div>
          </div>
        </div>

        <div class="wildcard-card">
          <div class="wildcard-symbol">[]</div>
          <div class="wildcard-name">Corchetes</div>
          <div class="wildcard-desc">
            Representa <strong>uno de</strong> los caracteres listados
          </div>
          <div class="wildcard-examples">
            <div class="example-item"><code>log[123].txt</code> → log1.txt, log2.txt, log3.txt</div>
            <div class="example-item">
              <code>[A-Z]*.py</code> → Archivos que empiezan con mayúscula
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Ejemplos Prácticos con Wildcards</SectionTitle>
        <div class="row q-col-gutter-md q-mt-sm">
          <div class="col-12 col-md-6">
            <CodeBlock
              title="Copiar todos los archivos Python a backup"
              lang="bash"
              content="cp *.py backup/"
              :copyable="true"
            />
          </div>
          <div class="col-12 col-md-6">
            <CodeBlock
              title="Borrar todos los logs temporales"
              lang="bash"
              content="rm temp_*.log"
              :copyable="true"
            />
          </div>
          <div class="col-12 col-md-6">
            <CodeBlock
              title="Mover archivos de configuración"
              lang="bash"
              content="mv *.yaml config/"
              :copyable="true"
            />
          </div>
          <div class="col-12 col-md-6">
            <CodeBlock
              title="Listar solo archivos de rosbag"
              lang="bash"
              content="ls *.db3"
              :copyable="true"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- ============================================ -->
    <!-- SECCIÓN 8: ERRORES COMUNES -->
    <!-- ============================================ -->
    <div class="section-group">
      <SectionTitle>Errores Comunes y Soluciones</SectionTitle>

      <div class="error-accordion">
        <q-expansion-item
          icon="error"
          label="mkdir: cannot create directory 'carpeta': File exists"
          header-class="error-header"
        >
          <div class="error-content">
            <div class="error-cause">
              <strong>Causa:</strong> Intentaste crear una carpeta que ya existe.
            </div>
            <div class="error-solution">
              <strong>Solución:</strong>
              <ul>
                <li>Usa <code>ls</code> para verificar qué carpetas existen</li>
                <li>Usa <code>mkdir -p</code> que no da error si ya existe</li>
                <li>Elige otro nombre para la carpeta</li>
              </ul>
            </div>
          </div>
        </q-expansion-item>

        <q-expansion-item
          icon="error"
          label="cp: cannot stat 'archivo.txt': No such file or directory"
          header-class="error-header"
        >
          <div class="error-content">
            <div class="error-cause">
              <strong>Causa:</strong> El archivo origen no existe o escribiste mal el nombre.
            </div>
            <div class="error-solution">
              <strong>Solución:</strong>
              <ol>
                <li>Verifica que estás en el directorio correcto con <code>pwd</code></li>
                <li>Lista los archivos disponibles con <code>ls</code></li>
                <li>Usa TAB para autocompletar nombres</li>
                <li>Recuerda que Linux es case-sensitive (Archivo.txt ≠ archivo.txt)</li>
              </ol>
            </div>
          </div>
        </q-expansion-item>

        <q-expansion-item
          icon="error"
          label="rm: cannot remove 'carpeta': Is a directory"
          header-class="error-header"
        >
          <div class="error-content">
            <div class="error-cause">
              <strong>Causa:</strong> Intentaste borrar una carpeta sin la bandera <code>-r</code>.
            </div>
            <div class="error-solution">
              <strong>Solución:</strong>
              <ul>
                <li>Usa <code>rm -r carpeta/</code> para borrar la carpeta y su contenido</li>
                <li>Usa <code>rm -ri carpeta/</code> si quieres confirmación para cada archivo</li>
                <li>
                  Alternativamente, usa <code>rmdir carpeta/</code> (solo funciona si está vacía)
                </li>
              </ul>
            </div>
          </div>
        </q-expansion-item>

        <q-expansion-item
          icon="error"
          label="Borré un archivo importante por accidente, ¿puedo recuperarlo?"
          header-class="error-header"
        >
          <div class="error-content">
            <div class="error-cause">
              <strong>Situación:</strong> Ejecutaste <code>rm</code> y te arrepentiste
              inmediatamente.
            </div>
            <div class="error-solution">
              <strong>Respuesta dura:</strong> Si usaste <code>rm</code>, el archivo está perdido
              para siempre. No hay papelera de reciclaje. <br /><br />
              <strong>Prevención futura:</strong>
              <ul>
                <li>Usa Git para versionar tu código (puedes recuperar versiones antiguas)</li>
                <li>
                  Haz backups regulares con <code>cp</code> o herramientas como <code>rsync</code>
                </li>
                <li>
                  Configura el alias <code>alias rm='rm -i'</code> para confirmación automática
                </li>
                <li>
                  Considera usar <code>trash-cli</code> en lugar de <code>rm</code> para tener
                  papelera
                </li>
              </ul>
            </div>
          </div>
        </q-expansion-item>
      </div>
    </div>

    <!-- ============================================ -->
    <!-- SECCIÓN 9: RETO PRÁCTICO -->
    <!-- ============================================ -->
    <div class="section-group">
      <SectionTitle>🏆 Reto de Ingeniería: Organizar un Workspace</SectionTitle>
      <TextBlock>
        Simula la creación de un workspace de ROS 2 desde cero. Ejecuta estos comandos línea por
        línea y observa cómo se construye la estructura:
      </TextBlock>

      <CodeBlock
        title="Construcción de Workspace Completo"
        lang="bash"
        content="# 1. Crear estructura base
mkdir -p ~/ros2_ws/src/mi_robot/{launch,config,scripts,msg}

# 2. Crear archivos de configuración
cd ~/ros2_ws/src/mi_robot
touch package.xml setup.py
touch config/nav2_params.yaml
touch launch/robot_bringup.launch.py
touch scripts/control_node.py

# 3. Crear mensaje custom
touch msg/SensorData.msg

# 4. Verificar estructura
cd ~/ros2_ws
tree src/  # (o usa 'find src/' si no tienes tree)

# 5. Hacer backup de la configuración
cp config/nav2_params.yaml config/nav2_params.yaml.backup

# 6. Simular limpieza de archivos temporales
cd ~/ros2_ws
touch build.log install.log
rm *.log

# 7. Verificar resultado final
ls -lah"
        :copyable="true"
      />

      <div class="q-mt-md">
        <AlertBlock type="success" title="Resultado Esperado">
          Deberías tener una estructura completa de paquete ROS 2 con carpetas organizadas, archivos
          placeholder, y un backup de configuración. Los archivos <code>.log</code> temporales
          fueron eliminados.
        </AlertBlock>
      </div>
    </div>

    <!-- ============================================ -->
    <!-- SECCIÓN 10: VIDEO COMPLEMENTARIO -->
    <!-- ============================================ -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <TextBlock>
        Mira este video tutorial que demuestra estas operaciones de gestión de archivos en acción:
      </TextBlock>

      <div class="video-container q-mt-md">
        <div class="video-wrapper">
          <iframe
            src="https://youtu.be/Romc22GgusU"
            title="Gestión de Archivos en Linux"
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
          <q-icon name="info" color="blue-4" size="sm" class="q-mr-sm" />
          Reemplaza el ID del video (dQw4w9WgXcQ) en el código fuente con tu video real de YouTube
        </div>
      </div>
    </div>

    <!-- ============================================ -->
    <!-- SECCIÓN 11: RESUMEN -->
    <!-- ============================================ -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen de Comandos Esenciales</SectionTitle>

      <div class="summary-table">
        <div class="summary-row summary-header">
          <div class="summary-cell">Comando</div>
          <div class="summary-cell">Descripción</div>
          <div class="summary-cell">Ejemplo</div>
        </div>
        <div class="summary-row">
          <div class="summary-cell"><code>mkdir -p</code></div>
          <div class="summary-cell">Crear directorios (con jerarquía)</div>
          <div class="summary-cell"><code>mkdir -p a/b/c</code></div>
        </div>
        <div class="summary-row">
          <div class="summary-cell"><code>touch</code></div>
          <div class="summary-cell">Crear archivo vacío</div>
          <div class="summary-cell"><code>touch robot.py</code></div>
        </div>
        <div class="summary-row">
          <div class="summary-cell"><code>cp -r</code></div>
          <div class="summary-cell">Copiar archivos/directorios</div>
          <div class="summary-cell"><code>cp -r src/ backup/</code></div>
        </div>
        <div class="summary-row">
          <div class="summary-cell"><code>mv</code></div>
          <div class="summary-cell">Mover o renombrar</div>
          <div class="summary-cell"><code>mv old.txt new.txt</code></div>
        </div>
        <div class="summary-row">
          <div class="summary-cell"><code>rm -r</code></div>
          <div class="summary-cell">Eliminar archivos/directorios</div>
          <div class="summary-cell"><code>rm -r temp/</code></div>
        </div>
        <div class="summary-row">
          <div class="summary-cell"><code>*.ext</code></div>
          <div class="summary-cell">Wildcard para múltiples archivos</div>
          <div class="summary-cell"><code>rm *.log</code></div>
        </div>
      </div>

      <AlertBlock type="success" title="Próximo Paso" class="q-mt-lg">
        Ahora que dominas la creación y gestión de archivos, el siguiente módulo te enseñará sobre
        <strong>permisos y usuarios</strong>: cómo controlar quién puede leer, escribir o ejecutar
        tus archivos. ¡Esencial para la seguridad de tu robot!
      </AlertBlock>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import LessonContainer from 'components/content/LessonContainer.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
</script>

<style scoped>
/* ============================================ */
/* LAYOUT GENERAL */
/* ============================================ */
.section-group {
  margin-bottom: 3.5rem;
}

/* ============================================ */
/* HIERARCHY DEMO */
/* ============================================ */
.hierarchy-demo {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.9), rgba(30, 41, 59, 0.9));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 1.5rem;
  height: 100%;
}

.demo-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #60a5fa;
  margin-bottom: 1rem;
}

.demo-note {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-top: 1rem;
  padding: 0.75rem;
  background: rgba(234, 179, 8, 0.1);
  border-radius: 8px;
  color: #fbbf24;
  font-size: 0.9rem;
}

/* ============================================ */
/* CONCEPT CARDS */
/* ============================================ */
.concept-card {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.9), rgba(30, 41, 59, 0.9));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 1.5rem;
  height: 100%;
  display: flex;
  flex-direction: column;
}

.concept-header {
  display: flex;
  align-items: center;
  gap: 12px;
  margin-bottom: 1.5rem;
}

.concept-title {
  font-size: 1.25rem;
  font-weight: 700;
  color: #f1f5f9;
}

.concept-body {
  flex-grow: 1;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

/* ============================================ */
/* SYNTAX CARD */
/* ============================================ */
.syntax-card {
  background: rgba(15, 23, 42, 0.8);
  border: 1px solid rgba(59, 130, 246, 0.3);
  border-radius: 12px;
  padding: 1.5rem;
}

.syntax-label {
  font-size: 0.85rem;
  color: #94a3b8;
  margin-bottom: 0.75rem;
  text-transform: uppercase;
  letter-spacing: 0.05em;
}

.syntax-pattern {
  font-family: 'Fira Code', monospace;
  font-size: 1.25rem;
  display: flex;
  align-items: center;
  gap: 12px;
  flex-wrap: wrap;
}

.syntax-cmd {
  color: #22c55e;
  font-weight: 700;
}

.syntax-arg {
  color: #94a3b8;
  padding: 0.25rem 0.75rem;
  background: rgba(255, 255, 255, 0.05);
  border-radius: 6px;
}

.syntax-arg.origin {
  color: #60a5fa;
  background: rgba(59, 130, 246, 0.1);
}

.syntax-arg.dest {
  color: #a78bfa;
  background: rgba(167, 139, 250, 0.1);
}

/* ============================================ */
/* OPERATION CARDS */
/* ============================================ */
.operation-card {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.8), rgba(30, 41, 59, 0.8));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  height: 100%;
  display: flex;
  flex-direction: column;
  align-items: center;
  text-align: center;
  transition:
    transform 0.2s,
    box-shadow 0.2s;
}

.operation-card:hover {
  transform: translateY(-4px);
  box-shadow: 0 8px 24px rgba(59, 130, 246, 0.2);
}

.operation-icon {
  margin-bottom: 1rem;
}

.operation-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.75rem;
}

.operation-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
  margin-bottom: 1rem;
  flex-grow: 1;
}

.operation-note {
  margin-top: 1rem;
  padding: 0.5rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 6px;
  font-size: 0.85rem;
  color: #94a3b8;
}

/* ============================================ */
/* MV COMPARISON */
/* ============================================ */
.mv-comparison {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.mv-case {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.mv-case-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  font-weight: 700;
  font-size: 1.1rem;
}

.mv-case-header.case-move {
  background: rgba(6, 182, 212, 0.1);
  border-bottom: 2px solid #06b6d4;
  color: #22d3ee;
}

.mv-case-header.case-rename {
  background: rgba(139, 92, 246, 0.1);
  border-bottom: 2px solid #8b5cf6;
  color: #a78bfa;
}

.mv-case-header.case-both {
  background: rgba(34, 197, 94, 0.1);
  border-bottom: 2px solid #22c55e;
  color: #4ade80;
}

.mv-case-body {
  padding: 1.5rem;
}

/* ============================================ */
/* DANGER ZONE */
/* ============================================ */
.danger-zone {
  background: linear-gradient(135deg, rgba(127, 29, 29, 0.2), rgba(153, 27, 27, 0.2));
  border: 2px solid rgba(239, 68, 68, 0.5);
  border-radius: 16px;
  overflow: hidden;
}

.danger-header {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 16px;
  padding: 1.5rem;
  background: rgba(127, 29, 29, 0.3);
  border-bottom: 2px solid rgba(239, 68, 68, 0.5);
}

.danger-title {
  font-size: 1.5rem;
  font-weight: 800;
  color: #fca5a5;
  text-transform: uppercase;
  letter-spacing: 0.05em;
}

.danger-body {
  padding: 1.5rem;
}

/* ============================================ */
/* RM CARDS */
/* ============================================ */
.rm-card {
  background: rgba(15, 23, 42, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.25rem;
  height: 100%;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.rm-header {
  display: flex;
  align-items: center;
  gap: 12px;
  color: #f1f5f9;
}

.rm-title {
  font-size: 1.1rem;
  font-weight: 700;
}

.rm-note {
  background: rgba(255, 255, 255, 0.03);
  padding: 0.75rem;
  border-radius: 6px;
  font-size: 0.85rem;
  color: #94a3b8;
}

/* ============================================ */
/* WILDCARDS GRID */
/* ============================================ */
.wildcards-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 1.5rem;
}

.wildcard-card {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.9), rgba(30, 41, 59, 0.9));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 1.5rem;
  text-align: center;
}

.wildcard-symbol {
  font-family: 'Fira Code', monospace;
  font-size: 3rem;
  font-weight: 700;
  color: #fbbf24;
  margin-bottom: 0.5rem;
}

.wildcard-name {
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.75rem;
}

.wildcard-desc {
  color: #cbd5e1;
  font-size: 0.95rem;
  margin-bottom: 1.5rem;
}

.wildcard-examples {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
  text-align: left;
}

.example-item {
  background: rgba(0, 0, 0, 0.3);
  padding: 0.75rem;
  border-radius: 8px;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
  color: #94a3b8;
  border-left: 3px solid #3b82f6;
}

/* ============================================ */
/* ERROR ACCORDION */
/* ============================================ */
.error-accordion {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
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
  border-radius: 0 0 8px 8px;
}

.error-cause {
  margin-bottom: 1rem;
  color: #fca5a5;
}

.error-solution {
  color: #cbd5e1;
}

.error-solution ol,
.error-solution ul {
  margin-top: 0.5rem;
  padding-left: 1.5rem;
}

.error-solution li {
  margin-bottom: 0.5rem;
}

/* ============================================ */
/* VIDEO CONTAINER */
/* ============================================ */
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
  margin-top: 1rem;
  padding: 0.75rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 8px;
  color: #94a3b8;
  font-size: 0.85rem;
}

/* ============================================ */
/* SUMMARY TABLE */
/* ============================================ */
.summary-table {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.summary-row {
  display: grid;
  grid-template-columns: 1fr 2fr 1.5fr;
  gap: 1rem;
  padding: 1rem 1.5rem;
  border-bottom: 1px solid rgba(148, 163, 184, 0.1);
}

.summary-row:last-child {
  border-bottom: none;
}

.summary-header {
  background: rgba(59, 130, 246, 0.1);
  font-weight: 700;
  color: #60a5fa;
}

.summary-cell {
  display: flex;
  align-items: center;
  color: #e2e8f0;
  font-size: 0.95rem;
}

.summary-cell code {
  background: rgba(0, 0, 0, 0.3);
  padding: 0.25rem 0.5rem;
  border-radius: 4px;
  font-family: 'Fira Code', monospace;
  color: #22c55e;
}

/* ============================================ */
/* RESPONSIVE */
/* ============================================ */
@media (max-width: 768px) {
  .syntax-pattern {
    font-size: 1rem;
  }

  .summary-row {
    grid-template-columns: 1fr;
    gap: 0.5rem;
  }

  .wildcards-grid {
    grid-template-columns: 1fr;
  }

  .mv-comparison {
    gap: 1rem;
  }
}
</style>
