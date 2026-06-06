<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        En C++, compilar significa "traducir código humano a código máquina". En Python, no se
        compila realmente, pero necesitamos "instalar" los scripts en la carpeta correcta
        (`install/`) para que ROS los encuentre.
        <br /><br />
        **Colcon** es la herramienta universal que maneja ambos lenguajes.
      </TextBlock>
    </div>

    <!-- 1. EL COMANDO BÁSICO -->
    <div class="section-group">
      <SectionTitle>1. El Martillazo: colcon build</SectionTitle>
      <TextBlock> Este es el comando que escribirás mil veces al día. </TextBlock>

      <CodeBlock
        title="Compilación Estándar"
        lang="bash"
        content="# Ir a la raíz del workspace (SIEMPRE)
cd ~/ros2_ws

# Construir todo
colcon build

# Salida esperada:
# Starting >>> mi_paquete
# Finished <<< mi_paquete [1.2s]
# Summary: 1 package finished [1.4s]"
        :copyable="true"
      />
    </div>

    <!-- 2. SYMLINK INSTALL (MAGIA PYTHON) -->
    <div class="section-group">
      <SectionTitle>2. El Truco Mágico: --symlink-install</SectionTitle>

      <AlertBlock type="success" title="Productividad x10">
        Por defecto, <code>colcon build</code> **copia** tus archivos Python a la carpeta
        <code>install</code>. Si modificas tu código en <code>src</code>, tienes que volver a
        compilar para actualizar la copia. ¡Qué pereza!
      </AlertBlock>

      <TextBlock>
        La bandera <code>--symlink-install</code> crea un "acceso directo" (enlace simbólico) en
        lugar de una copia.
        <br />
        Así, cualquier cambio que hagas en <code>src</code> se refleja
        <strong>inmediatamente</strong> en <code>install</code> sin tener que compilar de nuevo.
      </TextBlock>

      <CodeBlock
        title="Compilación Inteligente"
        lang="bash"
        content="colcon build --symlink-install"
        :copyable="true"
      />

      <p class="text-caption q-mt-sm text-grey-4">
        *Nota: Esto solo aplica a archivos Python, YAML y Launch. C++ siempre requiere
        recompilación.*
      </p>
    </div>

    <!-- 3. COMPILAR UN SOLO PAQUETE -->
    <div class="section-group">
      <SectionTitle>3. Compilación Quirúrgica</SectionTitle>
      <TextBlock>
        Cuando tengas 50 paquetes en tu robot, <code>colcon build</code> tardará 10 minutos. Si solo
        cambiaste un paquete, ¿para qué compilar los otros 49?
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="cmd-card">
            <div class="text-h6 text-primary">--packages-select</div>
            <p>Compila SOLO el paquete que le digas.</p>
            <code>colcon build --packages-select mi_paquete_robot</code>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="cmd-card">
            <div class="text-h6 text-secondary">--packages-up-to</div>
            <p>Compila el paquete Y sus dependencias (lo que necesita para funcionar).</p>
            <code>colcon build --packages-up-to mi_paquete_robot</code>
          </div>
        </div>
      </div>
    </div>

    <!-- 4. LIMPIAR EL DESASTRE -->
    <div class="section-group">
      <SectionTitle>4. Cuando todo falla: Clean</SectionTitle>
      <TextBlock>
        A veces `colcon` se confunde (especialmente si cambias nombres de archivos o dependencias).
        Si tienes errores extraños que no tienen sentido, la solución es:
        <strong>Borrón y Cuenta Nueva</strong>.
      </TextBlock>

      <CodeBlock
        title="Limpieza Radical"
        lang="bash"
        content="# Borrar las carpetas generadas
rm -rf build/ install/ log/

# Volver a compilar desde cero
colcon build --symlink-install"
      />

      <AlertBlock type="warning" title="Advertencia">
        Esto borrará cualquier configuración temporal, pero NO toca tu código fuente en
        <code>src</code>.
      </AlertBlock>
    </div>

    <!-- 5. VERIFICACIÓN (SOURCE) -->
    <div class="section-group">
      <SectionTitle>5. El Olvido Eterno: Source</SectionTitle>
      <TextBlock>
        Compilar crea los archivos, pero <strong>NO actualiza tu terminal actual</strong>. Linux no
        sabe que acabas de crear un ejecutable nuevo.
      </TextBlock>

      <div class="step-visual q-my-md">
        <div class="step">1. Modificar Código</div>
        <div class="arrow">➜</div>
        <div class="step">2. Colcon Build</div>
        <div class="arrow">➜</div>
        <div class="step highlight">3. Source install/setup.bash</div>
        <div class="arrow">➜</div>
        <div class="step">4. Ejecutar</div>
      </div>

      <TextBlock>
        Si olvidas el paso 3, ejecutarás la versión **VIEJA** de tu código y te volverás loco
        buscando el error.
        <br />
        <em>Tip: Usa el alias <code>s</code> que creamos en la lección anterior.</em>
      </TextBlock>
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto Constructor</SectionTitle>
      <StepsBlock
        :steps="[
          'Ve a tu workspace: cd ~/ros2_ws',
          'Ejecuta colcon build --symlink-install',
          'Verifica que terminó sin errores (Summary: X packages finished).',
          'Haz source install/setup.bash',
          'Comprueba que tu entorno ve el workspace: printenv | grep ROS_PACKAGE_PATH',
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
// import SplitBlock from 'components/content/SplitBlock.vue';
import StepsBlock from 'components/content/StepsBlock.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3rem;
}

.cmd-card {
  background: var(--bg-surface-solid);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  height: 100%;
}
.cmd-card code {
  display: block;
  background: var(--bg-surface-hover);
  padding: 8px;
  border-radius: 4px;
  margin-top: 10px;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
  color: var(--text-warning, #d97706);
  word-break: break-all;
}

/* Visualización de Pasos */
.step-visual {
  display: flex;
  justify-content: center;
  align-items: center;
  flex-wrap: wrap;
  gap: 10px;
}
.step {
  background: var(--bg-surface-solid);
  padding: 8px 16px;
  border-radius: 6px;
  font-weight: bold;
  font-size: 0.9rem;
}
.step.highlight {
  background: #ef4444; /* Rojo para resaltar */
  animation: pulse 2s infinite;
}
.arrow {
  color: var(--text-muted);
  font-size: 1.2rem;
}

@keyframes pulse {
  0% {
    box-shadow: 0 0 0 0 rgba(239, 68, 68, 0.7);
  }
  70% {
    box-shadow: 0 0 0 10px rgba(239, 68, 68, 0);
  }
  100% {
    box-shadow: 0 0 0 0 rgba(239, 68, 68, 0);
  }
}
</style>
