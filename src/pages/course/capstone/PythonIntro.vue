<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        Si Linux es el suelo donde camina tu robot, <strong>Python</strong> es el idioma que habla
        su cerebro. <br /><br />
        En ROS 2, usamos Python para escribir "Nodos": pequeños programas que controlan motores,
        leen cámaras o toman decisiones. Lo mejor de Python es que se lee casi como inglés. No
        necesitas compilar (traducir a máquina) antes de probar, lo que hace el desarrollo muy
        rápido.
      </TextBlock>
    </div>

    <!-- 1. LA REGLA DE ORO: INDENTACIÓN -->
    <div class="section-group">
      <SectionTitle>1. La Regla de Oro: Indentación</SectionTitle>

      <AlertBlock type="danger" title="⚠️ Cuidado Crítico">
        A diferencia de C++ o Java que usan llaves <code>{}</code> para agrupar código, Python usa
        <strong>espacios en blanco (sangría)</strong>.
        <br />
        Si mueves una línea de código un espacio a la derecha, cambia totalmente su significado.
      </AlertBlock>

      <SplitBlock>
        <template #left>
          <div class="text-weight-bold text-positive q-mb-sm">✅ Código Correcto</div>
          <TextBlock>
            La línea <code>print</code> está dentro del <code>if</code> porque tiene 4 espacios de
            margen.
          </TextBlock>
          <CodeBlock
            lang="python"
            content="bateria = 10

if bateria < 20:
    print('¡Batería baja!')
    print('Regresando a base...')"
          />
        </template>
        <template #right>
          <div class="text-weight-bold text-negative q-mb-sm">❌ Error de Sintaxis</div>
          <TextBlock>
            Aquí Python se confunde. La segunda línea no está alineada. Esto generará un
            <code>IndentationError</code>.
          </TextBlock>
          <CodeBlock
            lang="python"
            content="bateria = 10

if bateria < 20:
    print('¡Batería baja!')
  print('Regresando a base...') # ERROR"
          />
        </template>
      </SplitBlock>
    </div>

    <!-- 2. VARIABLES Y TIPOS (SENSORES) -->
    <div class="section-group">
      <SectionTitle>2. Variables: La Memoria del Robot</SectionTitle>
      <TextBlock>
        Una variable es como una caja con una etiqueta donde guardas datos. En robótica, los tipos
        de datos son cruciales.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="concept-card">
            <div class="text-h6 text-primary">Tipos Comunes</div>
            <ul class="simple-list">
              <li><strong>Integer (int):</strong> Números enteros. Ej: Ruedas (4).</li>
              <li><strong>Float (float):</strong> Decimales. Ej: Velocidad (1.5 m/s).</li>
              <li><strong>String (str):</strong> Texto. Ej: Mensajes ("Iniciando").</li>
              <li><strong>Boolean (bool):</strong> Verdad/Falso. Ej: ¿Hay obstáculo? (True).</li>
            </ul>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <CodeBlock
            title="ejemplo_variables.py"
            lang="python"
            content="num_ruedas = 4          # int
voltaje = 12.5          # float
nombre_robot = 'Wall-E' # str
esta_encendido = True   # bool

print(f'El robot {nombre_robot} tiene {voltaje}V')"
          />
        </div>
      </div>
    </div>

    <!-- 3. ESTRUCTURAS DE CONTROL (LÓGICA) -->
    <div class="section-group">
      <SectionTitle>3. La Lógica del Robot (if/else)</SectionTitle>
      <TextBlock>
        Los robots necesitan tomar decisiones basadas en sus sensores. Para eso usamos
        <code>if</code> (si), <code>elif</code> (si no, pero si...) y <code>else</code> (si no).
      </TextBlock>

      <CodeBlock
        title="evitar_obstaculos.py"
        lang="python"
        content="distancia = 0.5  # metros

if distancia < 0.2:
    accion = 'PARADA DE EMERGENCIA'
elif distancia < 1.0:
    accion = 'Reducir velocidad'
else:
    accion = 'Velocidad máxima'

print(f'Acción actual: {accion}')"
        :copyable="true"
      />
    </div>

    <!-- 4. BUCLES (LOOPS) -->
    <div class="section-group">
      <SectionTitle>4. Ciclos Infinitos (While)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Un script normal empieza y termina. Pero un <strong>Nodo de ROS</strong> suele funcionar
            para siempre (hasta que apagas el robot). <br /><br />
            Para esto usamos el ciclo <code>while True</code>. Es el corazón que mantiene vivo al
            programa.
          </TextBlock>
          <AlertBlock type="info" title="Interrumpir">
            Si creas un bucle infinito, el programa nunca parará solo. Debes presionar
            <strong>Ctrl + C</strong>
            en la terminal para matarlo.
          </AlertBlock>
        </template>
        <template #right>
          <CodeBlock
            title="bucle_control.py"
            lang="python"
            content="import time

bateria = 100

while bateria > 0:
    print(f'Robot patrullando... Batería: {bateria}%')
    bateria = bateria - 10
    time.sleep(1) # Esperar 1 segundo

print('Batería agotada. Apagando.')"
          />
        </template>
      </SplitBlock>
    </div>

    <!-- 5. SHEBANG Y EJECUCIÓN -->
    <div class="section-group">
      <SectionTitle>5. Ejecutar Scripts en Linux</SectionTitle>
      <TextBlock>
        Para ejecutar un archivo Python en Linux, hay un "ritual" especial para hacerlo
        profesionalmente.
      </TextBlock>

      <StepsBlock
        :steps="[
          'Crear el archivo: touch mi_robot.py',
          'Añadir el Shebang: La primera línea debe ser #!/usr/bin/env python3',
          'Dar permisos de ejecución: chmod +x mi_robot.py',
          'Ejecutarlo directamente: ./mi_robot.py',
        ]"
      />

      <div class="q-mt-md">
        <AlertBlock type="warning" title="¿Qué es el Shebang?">
          La línea <code>#!/usr/bin/env python3</code> le dice a la terminal de Linux: "Oye, no
          intentes leer esto tú mismo, búscame al intérprete de Python 3 y pásale este archivo".
        </AlertBlock>
      </div>
    </div>

    <!-- RETO FINAL -->
    <div class="section-group">
      <SectionTitle>🏆 Reto: Cuenta Regresiva</SectionTitle>
      <TextBlock>
        Crea un script llamado <code>lanzamiento.py</code> que haga lo siguiente:
      </TextBlock>

      <div class="q-pl-md q-mb-md">
        <ul class="text-grey-4">
          <li>Use un bucle para contar desde 5 hasta 1.</li>
          <li>Imprima cada número.</li>
          <li>Al final, imprima "¡Despegue!".</li>
        </ul>
      </div>

      <CodeBlock
        title="Solución (¡Inténtalo tú primero!)"
        lang="python"
        content="#!/usr/bin/env python3
import time

contador = 5

while contador > 0:
    print(f'T-minus: {contador}')
    contador = contador - 1
    time.sleep(1)

print('¡DESPEGUE! 🚀')"
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

.concept-card {
  background: var(--bg-surface-solid);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  height: 100%;
}

.simple-list {
  padding-left: 1.2rem;
  color: var(--text-secondary);
  line-height: 1.8;
}
.simple-list li strong {
  color: #38bdf8; /* Azul claro */
}

/* Colores semánticos locales */
.text-positive {
  color: #4ade80;
}
.text-negative {
  color: #f87171;
}
</style>
