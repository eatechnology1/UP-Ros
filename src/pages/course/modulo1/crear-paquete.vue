<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        Un **Paquete** es el contenedor de tu código. Es como una aplicación en tu celular. Puede
        contener nodos, configuraciones, mapas o definiciones de mensajes.
        <br /><br />
        En ROS 2, podemos crear paquetes para C++ (CMake) o para Python (ament_python). Nos
        centraremos en **Python**.
      </TextBlock>
    </div>

    <!-- 1. EL COMANDO GENERADOR -->
    <div class="section-group">
      <SectionTitle>1. El Comando Mágico (ros2 pkg create)</SectionTitle>
      <TextBlock>
        No creamos las carpetas y archivos a mano (sería propenso a errores). Usamos un comando
        generador que crea la estructura perfecta por nosotros.
      </TextBlock>

      <AlertBlock type="warning" title="Ubicación Crítica">
        Debes ejecutar este comando DENTRO de la carpeta <code>src</code> de tu workspace.
        <br />
        Ruta: <code>~/ros2_ws/src</code>
      </AlertBlock>

      <CodeBlock
        title="Creando el paquete"
        lang="bash"
        content="# 1. Ir a la fuente
cd ~/ros2_ws/src

# 2. Ejecutar el generador
ros2 pkg create --build-type ament_python --node-name mi_primer_nodo nombre_del_paquete"
      />

      <div class="q-mt-md">
        <div class="text-subtitle2 text-primary">Desglosando los argumentos:</div>
        <ul class="argument-list">
          <li>
            <code>--build-type ament_python</code>: Indica que usaremos Python (setup.py). Si lo
            omites, crea uno de C++ (CMake).
          </li>
          <li>
            <code>--node-name mi_primer_nodo</code>: (Opcional) Crea automáticamente un archivo
            "Hola Mundo" listo para correr.
          </li>
          <li>
            <code>nombre_del_paquete</code>: El nombre de la carpeta (sin espacios, usar guiones
            bajos).
          </li>
        </ul>
      </div>
    </div>

    <!-- 2. ANATOMÍA DEL PAQUETE -->
    <div class="section-group">
      <SectionTitle>2. Anatomía de un Paquete Python</SectionTitle>
      <TextBlock>
        Veamos qué creó el comando. Entra a la carpeta: <code>cd nombre_del_paquete</code>.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="file-card critical">
            <div class="icon">📄</div>
            <div class="title">package.xml</div>
            <p>
              El DNI del paquete. Define nombre, versión, autor, email y
              <strong>dependencias</strong> (qué otros paquetes necesita).
            </p>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="file-card critical">
            <div class="icon">⚙️</div>
            <div class="title">setup.py</div>
            <p>
              El instalador. Dice a ROS dónde poner tus scripts y cuál es el "punto de entrada"
              (main function).
            </p>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="file-card folder">
            <div class="icon">📂</div>
            <div class="title">nombre_del_paquete/</div>
            <p>
              Una subcarpeta con el MISMO nombre. <strong>Aquí va tu código fuente (.py)</strong>.
              Contiene un archivo <code>__init__.py</code>.
            </p>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="file-card folder">
            <div class="icon">📂</div>
            <div class="title">resource/ y test/</div>
            <p>Carpetas para metadatos y pruebas unitarias. Por ahora, ignóralas.</p>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. CONFIGURAR SETUP.PY -->
    <div class="section-group">
      <SectionTitle>3. El archivo setup.py (Entry Points)</SectionTitle>
      <TextBlock>
        Este archivo es el puente entre tu código Python y el comando <code>ros2 run</code>. Si
        escribes un script genial pero no lo registras aquí, ROS no sabrá cómo ejecutarlo.
      </TextBlock>

      <CodeBlock
        title="setup.py"
        lang="python"
        content="entry_points={
    'console_scripts': [
        # Formato: 'nombre_ejecutable = paquete.archivo:funcion_main'
        'mi_primer_nodo = nombre_del_paquete.mi_primer_nodo:main',
    ],
},"
      />

      <AlertBlock type="info" title="Traducción">
        La línea de arriba dice: "Cuando el usuario escriba
        <code>ros2 run nombre_del_paquete mi_primer_nodo</code>, busca el archivo
        <code>mi_primer_nodo.py</code> dentro de la carpeta del paquete y ejecuta la función
        <code>main()</code>".
      </AlertBlock>
    </div>

    <!-- 4. CICLO DE VIDA DE DESARROLLO -->
    <div class="section-group">
      <SectionTitle>4. Tu primer flujo completo</SectionTitle>
      <TextBlock>
        Ahora vamos a unir todo lo aprendido en las lecciones anteriores (Workspace, Colcon, Source
        y Paquete).
      </TextBlock>

      <StepsBlock
        :steps="[
          'Ir a src: cd ~/ros2_ws/src',
          'Crear paquete: ros2 pkg create --build-type ament_python --node-name hola_mundo my_bot_pkg',
          'Volver a raíz: cd ~/ros2_ws',
          'Compilar: colcon build --symlink-install',
          'Actualizar entorno: source install/setup.bash',
          'Ejecutar: ros2 run my_bot_pkg hola_mundo',
        ]"
      />

      <div class="q-mt-md text-center">
        <p class="text-positive text-weight-bold">
          Si ves en la terminal: "Hi from my_bot_pkg.", ¡Felicidades! Has creado tu primer software
          robótico.
        </p>
      </div>
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto Personalización</SectionTitle>
      <TextBlock>
        1. Busca el archivo <code>package.xml</code>. 2. Edítalo y cambia: -
        <code>&lt;description&gt;</code>: Pon algo divertido. - <code>&lt;maintainer&gt;</code>: Pon
        tu nombre y correo real. - <code>&lt;license&gt;</code>: Cambia a "MIT" o "Apache-2.0". 3.
        No necesitas recompilar para esto (son metadatos), pero es buena práctica verificar que
        <code>colcon build</code> no se queja.
      </TextBlock>
    </div>
  </div>
</template>

<script setup lang="ts">
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import StepsBlock from 'components/content/StepsBlock.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3rem;
}

.argument-list {
  color: #cbd5e1;
  line-height: 1.8;
  font-size: 0.95rem;
}
.argument-list code {
  color: #fcd34d; /* Amber */
  font-family: 'Fira Code', monospace;
  font-weight: bold;
}

.file-card {
  background: rgba(30, 41, 59, 0.5);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.2rem;
  height: 100%;
  transition: transform 0.2s;
}
.file-card:hover {
  transform: translateY(-3px);
  border-color: #38bdf8;
}

.file-card.critical {
  border-left: 4px solid #ef4444;
} /* Red marker */
.file-card.folder {
  border-left: 4px solid #3b82f6;
} /* Blue marker */

.file-card .icon {
  font-size: 2rem;
  margin-bottom: 0.5rem;
}
.file-card .title {
  font-weight: bold;
  font-size: 1.1rem;
  color: #f8fafc;
  margin-bottom: 0.5rem;
  font-family: 'Fira Code', monospace;
}
.file-card p {
  font-size: 0.85rem;
  color: #94a3b8;
  line-height: 1.4;
}
</style>
