<template>
  <LessonContainer>
    <!-- ========================================================================================
         BLOQUE A: FUNDAMENTOS MATEMÁTICOS Y FÍSICOS
         Objetivo: Establecer la base doctoral de Dinámica de Cuerpos Rígidos y Cinemática.
         Densidad: Alta (Teoría + Interactividad + Fórmulas LaTeX)
         ======================================================================================== -->

    <!-- HERO SECTION: THE ROBOT DNA -->
    <div class="hero-section">
      <div class="hero-content">
        <h1 class="hero-title">
          URDF Modeling & <span class="gradient-text">Rigid Body Dynamics</span>
        </h1>
        <p class="hero-subtitle">
          El <strong>Universal Robot Description Format</strong> no es solo XML. Es la manifestación
          digital de las Leyes de Newton-Euler. Aquí transformamos masa, inercia y geometría en
          ecuaciones diferenciales que Gazebo resuelve 1000 veces por segundo.
        </p>
        <div class="hero-badges">
          <q-badge color="primary" label="Estándar IEEE" />
          <q-badge color="orange" label="Physics Engineering" />
          <q-badge color="purple" label="KDL Theory" />
        </div>
      </div>
      <div class="hero-viz">
        <!-- Abstract representation of a Kinematic Chain -->
        <div class="chain-node base">
          <div class="node-circle">Base</div>
          <div class="node-line"></div>
        </div>
        <div class="chain-node link1">
          <div class="node-circle">Link 1</div>
          <div class="node-line"></div>
        </div>
        <div class="chain-node link2">
          <div class="node-circle">Link 2</div>
          <div class="node-line"></div>
        </div>
        <div class="chain-node ee">
          <div class="node-circle">EE</div>
        </div>
      </div>
    </div>

    <!-- SECTION 1: THE PHYSICS OF INERTIA (DOCTORAL DEEP DIVE) -->
    <div class="section-group">
      <SectionTitle>1. Teoría del Tensor de Inercia</SectionTitle>

      <TextBlock>
        Para un estudiante de primer año, la masa es un escalar ($m = 5kg$). Para un ingeniero en
        robótica, la resistencia al movimiento rotacional es una matriz de $3 \times 3$ llamada
        <strong>Tensor de Inercia</strong>. Sin esto, tu simulación explotará numéricamente.
      </TextBlock>

      <!-- DIDACTIC BOX: BEGINNER EXPLANATION -->
      <div class="didactic-box beginner">
        <div class="box-icon">🎓</div>
        <div class="box-content">
          <div class="box-title">Explicación para Primer Semestre: El Patinador de Hielo</div>
          <p>
            Imagina un patinador girando. Cuando extiende los brazos, gira lento. Cuando los pega al
            cuerpo, gira rápido. Su <strong>masa</strong> ($kg$) no cambió. Lo que cambió fue cómo
            esa masa está <strong>distribuida</strong>
            respecto al eje de giro.
            <br /><br />
            En URDF, debemos decirle a Gazebo exactamente "dónde está la masa" en cada eje (X, Y,
            Z). Si te equivocas, el robot girará como si fuera de poliestireno o de plomo en lugares
            incorrectos.
          </p>
        </div>
      </div>

      <!-- MATH DEEP DIVE: THE INERTIA TENSOR -->
      <div class="math-deep-dive q-mt-xl">
        <div class="math-header"><q-icon name="functions" /> Definición Matemática Rigurosa</div>
        <div class="matrix-viz-container">
          <div class="equation-container">
            <span class="eq-var">I</span> <span class="eq-op">=</span>
            <div class="matrix-wrapper">
              <div class="matrix-grid-3x3">
                <div class="m-cell diagonal">I<sub>xx</sub></div>
                <div class="m-cell">I<sub>xy</sub></div>
                <div class="m-cell">I<sub>xz</sub></div>
                <div class="m-cell">I<sub>yx</sub></div>
                <div class="m-cell diagonal">I<sub>yy</sub></div>
                <div class="m-cell">I<sub>yz</sub></div>
                <div class="m-cell">I<sub>zx</sub></div>
                <div class="m-cell">I<sub>zy</sub></div>
                <div class="m-cell diagonal">I<sub>zz</sub></div>
              </div>
              <div class="bracket l-bracket"></div>
              <div class="bracket r-bracket"></div>
            </div>
          </div>
          <div class="matrix-explanation">
            <ul>
              <li>
                <strong>Diagonal (I<sub>xx</sub>, I<sub>yy</sub>, I<sub>zz</sub>):</strong> Momentos
                de inercia principales. Resistencia a girar sobre los ejes X, Y, Z. Deben ser
                positivos.
              </li>
              <li>
                <strong>Off-Diagonal (I<sub>xy</sub>...):</strong> Productos de inercia. Indican
                asimetría. Para formas simétricas alineadas con los ejes, son <strong>CERO</strong>.
              </li>
            </ul>
          </div>
        </div>

        <!-- INTERACTIVE CALCULATOR: PARALLEL AXIS THEOREM -->
        <div class="interactive-tool q-mt-lg">
          <div class="tool-header">
            <div class="tool-title">Calculadora de Teorema de Steiner (Parallel Axis)</div>
            <div class="tool-desc">
              A menudo tenemos la inercia en el Centro de Masa (CoM), pero el URDF pide la inercia
              relativa al Joint. Usamos: <i>I = I<sub>cm</sub> + m · d²</i>
            </div>
          </div>

          <div class="calculator-grid">
            <div class="input-col">
              <q-input
                v-model.number="calcMass"
                label="Masa (kg)"
                type="number"
                dense
                filled
                dark
              />
              <q-input
                v-model.number="calcDist"
                label="Distancia al Eje (m)"
                type="number"
                dense
                filled
                dark
              />
              <q-input
                v-model.number="calcIcm"
                label="Inercia en CoM (kg*m²)"
                type="number"
                dense
                filled
                dark
              />
            </div>
            <div class="result-col">
              <div class="result-label">Inercia Resultante (I<sub>new</sub>):</div>
              <div class="result-value">
                {{ (calcIcm + calcMass * calcDist * calcDist).toFixed(4) }}
              </div>
              <div class="result-unit">kg · m²</div>
            </div>
          </div>

          <div class="viz-feedback">
            <div class="viz-object box-cm">CoM</div>
            <div class="viz-axis" :style="{ left: 50 + calcDist * 50 + 'px' }">Eje</div>
            <div class="viz-line" :style="{ width: calcDist * 50 + 'px' }">d</div>
          </div>
        </div>
      </div>
    </div>

    <!-- SECTION 2: KINEMATIC CHAINS & KDL (DOCTORAL LAYER) -->
    <div class="section-group">
      <SectionTitle>2. Cadenas Cinemáticas y Grafos de Escena</SectionTitle>

      <TextBlock>
        Un robot en ROS 2 es un <strong>Directed Acyclic Graph (DAG)</strong>. Cada eslabón (`link`)
        es un nodo, y cada `joint` es una arista que define una transformada geométrica. Esta
        estructura es la base de <strong>TF2</strong> y la librería <strong>KDL</strong>.
      </TextBlock>

      <!-- CONCEPT VISUALIZER: HOMOGENEOUS TRANSFORMS -->
      <div class="concept-viz-container q-mt-md">
        <div class="viz-header">
          <q-icon name="hub" /> Anatomía de una Transformada Homogénea (T<sub>4x4</sub>)
        </div>

        <div class="matrix-4x4-grid">
          <div class="m-quadrant rotation">
            <div class="q-label">Rotación (R<sub>3x3</sub>)</div>
            <div class="q-desc">Orientación (Roll, Pitch, Yaw -> Quaternions)</div>
          </div>
          <div class="m-quadrant position">
            <div class="q-label">Traslación (P<sub>3x1</sub>)</div>
            <div class="q-desc">Posición (x, y, z)</div>
          </div>
          <div class="m-quadrant perspective">
            <div class="q-label">Perspec. (0<sub>1x3</sub>)</div>
            <div class="q-desc">0, 0, 0 (Para robótica)</div>
          </div>
          <div class="m-quadrant scale">
            <div class="q-label">Escala (1<sub>1x1</sub>)</div>
            <div class="q-desc">1 (No escalamos robots)</div>
          </div>
        </div>

        <CodeBlock
          title="Representación Matemática"
          lang="python"
          content="# Matriz de Transformación Homogénea
T =A [
  [r11, r12, r13, tx],  # Eje X + Pos X
  [r21, r22, r23, ty],  # Eje Y + Pos Y
  [r31, r32, r33, tz],  # Eje Z + Pos Z
  [  0,   0,   0,  1]   # Factor de escala
]"
          :copyable="true"
          class="q-mt-sm"
        />
      </div>

      <!-- INTERACTIVE DH PARAMETERS (DENAVIT-HARTENBERG) -->
      <div class="interactive-tool q-mt-xl">
        <div class="tool-header">
          <div class="tool-title">Denavit-Hartenberg (DH) Parameters</div>
          <div class="tool-desc">
            En un brazo robótico clásico, describimos la relación entre cada eslabón con solo 4
            parámetros. Esto es vital para entender cómo `robot_state_publisher` calcula la posición
            del efector final.
          </div>
        </div>

        <div class="dh-viz">
          <div class="dh-diagram">
            <!-- Simplified CSS representation of 2 links -->
            <div class="link-segment l1" :style="{ transform: `rotate(${dh_theta}deg)` }">
              <div class="joint-axis">z</div>
              <div class="link-body" :style="{ width: `${dh_a * 50}px` }">a (Length)</div>
            </div>
            <div
              class="link-segment l2"
              :style="{
                transform: `translateX(${dh_a * 50}px) rotate(${dh_alpha}deg) translateY(${-dh_d * 20}px)`,
              }"
            >
              <div class="link-body">Link 2</div>
            </div>
          </div>

          <div class="dh-controls">
            <div class="control-row">
              <label><i>&theta;</i> (Theta): Rotación en Z</label>
              <q-slider v-model="dh_theta" :min="-90" :max="90" color="purple" dense />
            </div>
            <div class="control-row">
              <label><i>d</i> (Offset): Desplazamiento en Z</label>
              <q-slider v-model="dh_d" :min="0" :max="5" color="blue" dense />
            </div>
            <div class="control-row">
              <label><i>a</i> (Length): Longitud común Normal</label>
              <q-slider v-model="dh_a" :min="1" :max="5" color="green" dense />
            </div>
            <div class="control-row">
              <label><i>&alpha;</i> (Alpha): Twist en X</label>
              <q-slider v-model="dh_alpha" :min="-90" :max="90" color="orange" dense />
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ========================================================================================
         BLOQUE B: ARQUITECTURA XML Y XACRO AVANZADO
         Objetivo: Dominar la sintaxis URDF y la metaprogramación con Xacro (Macro Language).
         Densidad: Alta (Código Comentado + Diagramas de Arquitectura)
         ======================================================================================== -->

    <!-- SECTION 3: ANATOMY OF A LINK (WHITE PAPER STYLE) -->
    <div class="section-group">
      <SectionTitle>3. Anatomía de un Link: El Estándar IEEE-754</SectionTitle>

      <TextBlock>
        Un <code>&lt;link&gt;</code> no es solo un dibujo 3D. Es una entidad física con propiedades
        dinámicas estrictas. Gazebo ignorará cualquier link que no tenga <strong>Inercia</strong> o
        cuya masa sea cero (excepto la base estática).
      </TextBlock>

      <div class="urdf-anatomy-grid q-mt-md">
        <!-- Visual Column -->
        <div class="anatomy-card visual">
          <div class="card-header"><q-icon name="visibility" /> &lt;visual&gt;</div>
          <div class="card-body">
            Es la "piel" del robot. Solo sirve para que Rviz y tus ojos lo vean.
            <ul>
              <li><strong>Geometry:</strong> Malla (.dae/.stl) o Primitiva (Cilindro).</li>
              <li><strong>Material:</strong> Color o Textura PBR.</li>
              <li><strong>Origin:</strong> Offset respecto al Link Frame.</li>
            </ul>
          </div>
        </div>

        <!-- Collision Column -->
        <div class="anatomy-card collision">
          <div class="card-header"><q-icon name="contacts" /> &lt;collision&gt;</div>
          <div class="card-body">
            Es el "cuerpo sólido" que toca otros objetos.
            <div class="warning-box">
              ⚠️ <strong>Performance Tip:</strong> Nunca uses la malla visual de alta resolución
              aquí. Usa primitivas o cascos convexos (Convex Hulls).
            </div>
          </div>
        </div>

        <!-- Inertial Column -->
        <div class="anatomy-card inertial">
          <div class="card-header"><q-icon name="psychology" /> &lt;inertial&gt;</div>
          <div class="card-body">
            El cerebro de la dinámica.
            <ul>
              <li><strong>Mass:</strong> Escalar (kg).</li>
              <li><strong>Inertia:</strong> Tensor 3x3 calculado en el Bloque A.</li>
              <li><strong>Origin:</strong> Centro de Masa (CoM). ¡Crítico!</li>
            </ul>
          </div>
        </div>
      </div>

      <CodeBlock
        title="link_template.urdf"
        lang="xml"
        content='<link name="forearm_link">
  <!-- 1. INERTIAL: Obligatorio para Simulación (Si falta, Gazebo crashea) -->
  <inertial>
    <origin xyz="0 0 0.15" rpy="0 0 0"/> <!-- Centro de Masa Desplazado -->
    <mass value="2.5"/> <!-- kg -->
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
  </inertial>

  <!-- 2. COLLISION: Geometría simplificada para el motor ODE (Performance) -->
  <collision>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.05" length="0.3"/> <!-- Primitiva > Mesh -->
    </geometry>
  </collision>

  <!-- 3. VISUAL: Malla de alta calidad para rendering (Estética) -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://my_robot/meshes/visual/forearm_high_res.dae"/>
    </geometry>
    <material name="IndustrialGrey"/>
  </visual>
</link>'
        :copyable="true"
        class="q-mt-lg"
      />
    </div>

    <!-- SECTION 4: ADVANCED XACRO RECURSION (DOCTORAL PATTERN) -->
    <div class="section-group">
      <SectionTitle>4. Metaprogramación con Xacro (Macro Language)</SectionTitle>

      <TextBlock>
        Escribir XML a mano para un robot de 6 ejes es un error. Usamos <strong>Xacro</strong> (XML
        Macros) para programar la generación del URDF. Implementaremos un algoritmo recursivo para
        generar una "Snake Robot" de $N$ segmentos automáticamente.
      </TextBlock>

      <div class="xacro-studio q-mt-md">
        <div class="studio-header"><q-icon name="code" /> Xacro Loop Pattern</div>

        <div class="split-view">
          <div class="code-pane">
            <div class="pane-label">Input: snake.xacro</div>
            <CodeBlock
              lang="xml"
              content='<!-- Propiedad Matemática -->
<xacro:property name="link_len" value="0.2" />

<!-- MACRO RECURSIVA -->
<xacro:macro name="create_segment" params="id parent">
  <!-- Condición de Parada (Si id > 0) -->
  <xacro:if value="${id > 0}">

    <!-- Generar Joint -->
    <joint name="joint_${id}" type="revolute">
      <parent link="${parent}"/>
      <child link="link_${id}"/>
      <origin xyz="${link_len} 0 0"/>
    </joint>

    <!-- Generar Link -->
    <link name="link_${id}">...</link>

    <!-- LLAMADA RECURSIVA (id - 1) -->
    <xacro:create_segment id="${id-1}" parent="link_${id}"/>
  </xacro:if>
</xacro:macro>

<!-- Iniciar Recursión (5 segmentos) -->
<xacro:create_segment id="5" parent="base_link"/>'
              :copyable="true"
            />
          </div>

          <div class="arrow-pane">
            <q-icon name="arrow_forward" />
            <div class="process-label">xacro process</div>
          </div>

          <div class="code-pane output">
            <div class="pane-label">Output: robot.urdf</div>
            <div class="xml-tree">
              <div class="tag">&lt;joint name="joint_5"&gt;</div>
              <div class="tag">&lt;link name="link_5"&gt;</div>
              <div class="tag indent">&lt;joint name="joint_4"&gt;</div>
              <div class="tag indent">&lt;link name="link_4"&gt;</div>
              <div class="tag indent-2">... (x1000 lines avoided)</div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- SECTION 5: COLLISION OPTIMIZATION (ENGINEERING REALITY) -->
    <div class="section-group">
      <SectionTitle>5. Optimización de Colisiones: Mesh vs Convex Hull</SectionTitle>
      <AlertBlock type="warning" title="El Asesino del RTF (Real Time Factor)">
        El motor de física (ODE) calcula intersecciones triángulo por triángulo (`TriMesh`). Si usas
        una malla de 100k polígonos para collision, tu simulación correrá a 0.01x de la velocidad
        real.
      </AlertBlock>

      <div class="collision-viz-container q-mt-md">
        <div class="col-card bad">
          <div class="col-title">❌ Raw Mesh</div>
          <div class="col-viz">
            <!-- CSS Polygon representation -->
            <svg viewBox="0 0 100 100" class="col-svg">
              <path
                d="M10,90 L20,10 L40,30 L60,10 L80,90 Z"
                fill="none"
                stroke="#ef4444"
                stroke-width="1"
              />
              <path d="M20,10 L80,90" stroke="#ef4444" stroke-width="0.5" />
              <path d="M10,90 L40,30" stroke="#ef4444" stroke-width="0.5" />
            </svg>
          </div>
          <div class="col-stats">
            <div>Polys: 25,000</div>
            <div>Cost: O(N²)</div>
          </div>
        </div>

        <div class="col-card ok">
          <div class="col-title">⚠️ Convex Hull</div>
          <div class="col-viz">
            <svg viewBox="0 0 100 100" class="col-svg">
              <polygon
                points="10,90 20,10 60,10 80,90"
                fill="rgba(251,191,36,0.2)"
                stroke="#f59e0b"
                stroke-width="2"
              />
            </svg>
          </div>
          <div class="col-stats">
            <div>Polys: 12 (Simplified)</div>
            <div>Cost: O(N)</div>
          </div>
        </div>

        <div class="col-card good">
          <div class="col-title">✅ Primitive</div>
          <div class="col-viz">
            <div class="primitive-box"></div>
          </div>
          <div class="col-stats">
            <div>Formula: Box/Cyl</div>
            <div>Cost: O(1)</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ========================================================================================
         BLOQUE C: PIPELINE DE INGENIERÍA (CAD TO URDF)
         Objetivo: Workflow profesional para exportar modelos de SolidWorks/Fusion360 a Gazebo.
         Densidad: Alta (Guías de Exportación + Corrección de Mallas + Transmisiones)
         ======================================================================================== -->

    <!-- SECTION 6: CAD EXPORT & MESH REPAIR -->
    <div class="section-group">
      <SectionTitle>6. El "Valley of Death" del CAD a la Simulación</SectionTitle>

      <TextBlock>
        El error #1 de los novatos es exportar un ensamblaje de 500MB desde SolidWorks y esperar que
        Gazebo funcione. No lo hará. Los sistemas CAD (Paramétricos) y los Motores de Juego
        (Poligonales) hablan idiomas opuestos. Necesitas un
        <strong>Pipeline de Curado de Mallas</strong>.
      </TextBlock>

      <div class="pipeline-viz q-mt-md">
        <div class="p-step cad">
          <div class="p-icon"><q-icon name="architecture" /></div>
          <div class="p-label">1. SolidWorks / Fusion</div>
          <div class="p-desc">
            Exportar como <strong>.STEP</strong> (Geometría Pura) o usar Plugin URDF Exporter.
          </div>
        </div>
        <div class="p-arrow">➜</div>
        <div class="p-step blender">
          <div class="p-icon"><q-icon name="blender" /></div>
          <div class="p-label">2. Blender / MeshLab</div>
          <div class="p-desc">
            <ul>
              <li>Decimate (Reducir Polígonos).</li>
              <li>Bake Textures (PBR).</li>
              <li><strong>Reset Origin (0,0,0).</strong></li>
            </ul>
          </div>
        </div>
        <div class="p-arrow">➜</div>
        <div class="p-step sim">
          <div class="p-icon"><q-icon name="smart_toy" /></div>
          <div class="p-label">3. Gazebo (DAE/STL)</div>
          <div class="p-desc">Importar .DAE (Visual) y .STL (Collision). Verificar Escala.</div>
        </div>
      </div>

      <!-- MESH LAB: COMMON ERRORS -->
      <div class="mesh-lab-container q-mt-xl">
        <div class="lab-title">🔬 Laboratorio Forense de Mallas</div>
        <div class="errors-grid">
          <div class="error-card invert">
            <div class="e-header">Inverted Normals</div>
            <div class="e-viz">
              <div class="face front">Front</div>
              <div class="face back">Back</div>
            </div>
            <div class="e-fix">
              <strong>Síntoma:</strong> El robot se ve invisible o "hueco" en Gazebo.
              <br />
              <strong>Fix:</strong> Blender > Edit Mode > Mesh > Normals > Recalculate Outside.
            </div>
          </div>

          <div class="error-card origin">
            <div class="e-header">Origin Offset</div>
            <div class="e-viz">
              <div class="origin-point">0,0,0</div>
              <div class="ghost-mesh">Mesh</div>
            </div>
            <div class="e-fix">
              <strong>Síntoma:</strong> El link flota a 5 metros de su joint.
              <br />
              <strong>Fix:</strong> Apply Transform (Ctrl+A) en Blender antes de exportar.
            </div>
          </div>

          <div class="error-card scale">
            <div class="e-header">Units Mismatch</div>
            <div class="e-viz">
              <div class="measure big">1m (CAD)</div>
              <div class="measure small">1mm (Sim)</div>
            </div>
            <div class="e-fix">
              <strong>Síntoma:</strong> Robot gigante o microscópico (Explota física).
              <br />
              <strong>Fix:</strong> Verificar exportación (Metros vs Milímetros). STL no tiene
              unidades.
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- SECTION 7: TRANSMISSIONS & ROS2 CONTROL -->
    <div class="section-group">
      <SectionTitle>7. Transmisiones y Actuadores (Hardware Interface)</SectionTitle>

      <TextBlock>
        El URDF define la física, pero <code>ros2_control</code> define la electrónica y el control.
        Debemos etiquetar cada Joint con una <strong>Transmisión</strong> para mapear el esfuerzo
        del motor (Torque) al movimiento de la articulación.
      </TextBlock>

      <div class="transmission-architecture q-mt-md">
        <div class="arch-diagram">
          <div class="arch-block controller">
            <div class="a-title">ROS 2 Controller</div>
            <div class="a-sub">Joint Trajectory Controller</div>
            <div class="data-flow">Command (Pos/Vel)</div>
          </div>

          <div class="arch-block interface">
            <div class="a-title">Hardware Interface</div>
            <div class="a-sub">System / Actuator / Sensor</div>
            <CodeBlock
              lang="xml"
              content="<hardware>
  <plugin>gazebo_ros2_control/GazeboSystem</plugin>
</hardware>"
              class="mini-code"
            />
          </div>

          <div class="arch-block transmission">
            <div class="a-title">Transmission</div>
            <div class="a-sub">Simple / Differential</div>
            <div class="gear-viz">⚙️ Reduction Ratio</div>
          </div>

          <div class="arch-block joint">
            <div class="a-title">Joint (URDF)</div>
            <div class="a-sub">Revolute / Prismatic</div>
          </div>
        </div>

        <CodeBlock
          title="transmission_macro.xacro"
          lang="xml"
          content='<!-- Macro para inyectar transmisiones automáticamente -->
<xacro:macro name="setup_transmission" params="joint_name interface">
  <ros2_control name="${joint_name}_ctrl" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="${joint_name}">
      <!-- Command Interface: Lo que ordenamos (Posición) -->
      <command_interface name="${interface}">
        <param name="min">-3.14</param>
        <param name="max">3.14</param>
      </command_interface>
      <!-- State Interface: Lo que leemos (Pos, Vel, Esfuerzo) -->
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
</xacro:macro>'
          :copyable="true"
          class="q-mt-lg"
        />
      </div>
    </div>

    <!-- BLOCK C COMPLETION MARKER (Completed) -->
    <div class="block-completion completed">
      <q-separator spaced />
      <div class="completion-msg">
        <q-icon name="check_circle" color="green" />
        Bloque C (Pipeline & Transmisiones) Completado.
      </div>
    </div>

    <!-- ========================================================================================
         BLOQUE D: CASE STUDY (TITAN-6DOF INDUSTRIAL ARM)
         Objetivo: Unificar Física, Xacro y Pipeline en un producto final de ingeniería.
         Densidad: Máxima (Blueprint Interactivo + Código Final)
         ======================================================================================== -->

    <div class="section-group">
      <SectionTitle>8. Case Study: Titan-6DOF Industrial Arm</SectionTitle>

      <TextBlock>
        Aplicaremos todo lo aprendido para construir el <strong>Titan-6DOF</strong>, un manipulador
        industrial diseñado para soldadura de precisión. Este URDF final integra macros de inercia,
        mallas optimizadas, transmisiones de hardware y plugins de Gazebo.
      </TextBlock>

      <!-- BLUEPRINT VIZ -->
      <div class="blueprint-container q-mt-md">
        <div class="bp-header">
          <div class="bp-title">TITAN-6DOF ARCHITECTURE</div>
          <div class="bp-meta">REV: 2.0 | MATERIAL: ALLOY-7075 | PHYSICS: ODE-LCP</div>
        </div>

        <div class="bp-grid">
          <!-- Left: The Physical Robot (Abstract) -->
          <div class="bp-section robot-viz">
            <div class="robot-schematic">
              <div class="r-base">BASE</div>
              <div class="r-joint j1">J1</div>
              <div class="r-link l1">SHOULDER</div>
              <div class="r-joint j2">J2</div>
              <div class="r-link l2">ELBOW</div>
              <div class="r-joint j3">J3</div>
              <div class="r-ee">GRIPPER</div>
            </div>
          </div>

          <!-- Right: The Code Structure -->
          <div class="bp-section code-map">
            <div class="map-node root"><q-icon name="folder" /> titan_description</div>
            <div class="map-node file">
              <span class="file-icon">📄</span> robot.urdf.xacro
              <div class="node-desc">Entry Point (World + Robot)</div>
            </div>
            <div class="map-node file indented">
              <span class="file-icon">🧩</span> titan_core.xacro
              <div class="node-desc">Links & Joints (Macro Loop)</div>
            </div>
            <div class="map-node file indented">
              <span class="file-icon">⚙️</span> transmissions.xacro
              <div class="node-desc">ros2_control tags</div>
            </div>
            <div class="map-node file indented">
              <span class="file-icon">🧠</span> gazebo_plugins.xacro
              <div class="node-desc">libgazebo_ros2_control.so</div>
            </div>
          </div>
        </div>
      </div>

      <CodeBlock
        title="titan_core.xacro (Final Integration)"
        lang="xml"
        content='<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="titan_6dof">

  <!-- 1. INCLUDES: Traer nuestras herramientas -->
  <xacro:include filename="inertial_macros.xacro"/>
  <xacro:include filename="transmissions.xacro"/>

  <!-- 2. BASE LINK (Estático) -->
  <link name="base_link">
    <visual>
      <geometry><mesh filename="package://titan/meshes/base.dae"/></geometry>
    </visual>
    <collision>
      <geometry><cylinder radius="0.15" length="0.2"/></geometry>
    </collision>
    <inertial>
      <!-- Base masiva para estabilidad -->
      <mass value="50.0"/>
      <xacro:cylinder_inertia m="50" r="0.15" h="0.2"/>
    </inertial>
  </link>

  <!-- 3. SHOULDER (Dinámico) -->
  <link name="shoulder_link">
    <!-- Visual Bonito, Colisión Barata -->
    <visual>
      <geometry><mesh filename="package://titan/meshes/shoulder.dae"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.1 0.1 0.5"/></geometry>
    </collision>
    <!-- Inercia Rigurosa -->
    <xacro:inertial_box mass="5.0" x="0.1" y="0.1" z="0.5">
      <origin xyz="0 0 0.25"/> <!-- CoM desplazado -->
    </xacro:inertial_box>
  </link>

  <!-- 4. JOINT (Transmisión Inmediata) -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="300" velocity="1.0"/>
    <dynamics damping="0.7" friction="0.5"/> <!-- Física Realista -->
  </joint>

  <!-- INYECCIÓN AUTOMÁTICA DE ROS2 CONTROL -->
  <xacro:setup_transmission joint_name="shoulder_joint" interface="position"/>

</robot>'
        :copyable="true"
        class="q-mt-lg"
      />
    </div>

    <!-- CHALLENGE SECTION: THE EXPLODING ROBOT -->
    <div class="section-group">
      <SectionTitle>9. Debugging DOCTORAL: "¿Por qué explota mi robot?"</SectionTitle>

      <div class="challenge-container">
        <div class="challenge-header">
          <div class="c-icon">💥</div>
          <div class="c-info">
            <div class="c-title">Scenario: The Kepler Explosion</div>
            <div class="c-desc">
              Tu simulación inicia y el robot sale volando al infinito (NaN positions). Gazebo
              reporta violación de constraints. ¿Cuál es la causa raíz más probable?
            </div>
          </div>
        </div>

        <div class="challenge-options">
          <div class="c-option wrong">
            <div class="opt-radio">A</div>
            <div class="opt-text">El color de la malla visual es transparente.</div>
          </div>
          <div class="c-option wrong">
            <div class="opt-radio">B</div>
            <div class="opt-text">Olvidaste poner el plugin de `joint_state_publisher`.</div>
          </div>
          <div class="c-option correct">
            <div class="opt-radio">C</div>
            <div class="opt-text">
              <strong>Inercia Inválida:</strong> Matriz no definida positiva (ej: Ixx=0) o Masa=0 en
              un link dinámico. O colisión interna (Self-Collision) en posición inicial (t=0).
            </div>
            <div class="opt-feedback">
              ¡Correcto! El solver LCP (Linear Complementarity Problem) divide por la inercia. Si
              $I=0 \to \infty$ aceleración. Si dos collision shapes se solapan en $t=0$, generan
              fuerzas de repulsión infinitas.
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- SECTION 10: VIDEO TUTORIAL -->
    <div class="section-group">
      <SectionTitle>10. Video Tutorial: Construyendo a Titan paso a paso</SectionTitle>
      <div class="video-container q-mt-md">
        <div class="video-wrapper">
          <iframe
            width="100%"
            height="100%"
            src="https://youtu.be/Romc22GgusU"
            title="YouTube video player"
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
          <q-icon name="live_tv" />
          Acompaña al instructor en la construcción del archivo <code>titan_core.xacro</code> desde
          cero, depurando colisiones en vivo y configurando las transmisiones.
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen de Ingeniería</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>Inercia (Tensor)</code>
          <span>Matriz 3x3 fundamental para estabilidad física (Newton-Euler).</span>
        </div>
        <div class="summary-item">
          <code>Xacro</code>
          <span>Metaprogramación para generar URDF modular y mantenible.</span>
        </div>
        <div class="summary-item">
          <code>Collision</code>
          <span>Geometría simplificada (Primitivas/Hull) para evitar explosión de CPU.</span>
        </div>
        <div class="summary-item">
          <code>Transmisiones</code>
          <span>Puente crítico entre Hardware Interface y el Joint State.</span>
        </div>
        <div class="summary-item">
          <code>Visual vs Collision</code>
          <span>Separación estricta de dominios (Render vs Física).</span>
        </div>
      </div>

      <AlertBlock type="success" title="Best Practices: Zero-Explosion Protocol" class="q-mt-lg">
        ✅ <strong>Inercia:</strong> Calcular siempre en el Centro de Masa (CoM) exacto.
        <br />
        ✅ <strong>Collision:</strong> Jamás usar High-Poly meshes. Usar Primitivas > Convex Hulls.
        <br />
        ✅ <strong>Origin:</strong> Normalizar meshes externamente (Blender) a (0,0,0) y escala 1.0.
        <br />
        ✅ <strong>Joints:</strong> Verificar orientación de ejes en
        <code>solid_to_urdf</code> antes de exportar.
      </AlertBlock>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import { ref } from 'vue';
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';

// Reactive state for Parallel Axis Calculator
const calcMass = ref(10);
const calcDist = ref(0.2);
const calcIcm = ref(0.5);

// Reactive state for DH Parameters Viz
const dh_theta = ref(0);
const dh_d = ref(1);
const dh_a = ref(2);
const dh_alpha = ref(0);
</script>

<style scoped>
.section-group {
  margin-bottom: 5rem; /* Massive spacing for heavy content */
}

/* HERO SECTION - PREMIUM AESTHETIC */
.hero-section {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 4rem 2rem;
  background:
    radial-gradient(circle at 70% 50%, rgba(59, 130, 246, 0.15), transparent 60%),
    linear-gradient(180deg, #0f172a 0%, #1e293b 100%);
  border-radius: 20px;
  border: 1px solid rgba(148, 163, 184, 0.1);
  margin-bottom: 4rem;
  position: relative;
  overflow: hidden;
}

.hero-title {
  font-size: 3rem;
  font-weight: 800;
  line-height: 1.1;
  margin-bottom: 1.5rem;
  color: #fff;
  letter-spacing: -1px;
}

.gradient-text {
  background: linear-gradient(135deg, #60a5fa 0%, #c084fc 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
}

.hero-subtitle {
  font-size: 1.1rem;
  color: #94a3b8;
  max-width: 600px;
  line-height: 1.6;
  margin-bottom: 2rem;
}

.hero-badges {
  display: flex;
  gap: 1rem;
}

/* HERO ANIMATION */
.hero-viz {
  position: relative;
  width: 300px;
  height: 300px;
  display: flex;
  flex-direction: column;
  align-items: center;
}

.chain-node {
  display: flex;
  flex-direction: column;
  align-items: center;
  position: absolute;
  transition: all 0.5s ease-in-out;
}

.node-circle {
  width: 60px;
  height: 60px;
  border-radius: 50%;
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid #60a5fa;
  display: flex;
  align-items: center;
  justify-content: center;
  color: #60a5fa;
  font-weight: 700;
  z-index: 2;
  box-shadow: 0 0 20px rgba(96, 165, 250, 0.3);
}

.node-line {
  width: 4px;
  height: 60px;
  background: #334155;
  margin-top: -2px;
  z-index: 1;
}

/* Animation Keyframes would be complex, simplified for CSS here */
.chain-node.base {
  top: 220px;
}
.chain-node.link1 {
  top: 140px;
  animation: sway 3s infinite ease-in-out;
  transform-origin: bottom center;
}
.chain-node.link2 {
  top: 60px;
  animation: sway 3s infinite ease-in-out 0.5s;
  transform-origin: bottom center;
}
.chain-node.ee {
  top: 0px;
  animation: sway 3s infinite ease-in-out 1s;
  transform-origin: bottom center;
}

@keyframes sway {
  0%,
  100% {
    transform: rotate(-5deg);
  }
  50% {
    transform: rotate(5deg);
  }
}

/* DIDACTIC BOX */
.didactic-box.beginner {
  background: rgba(34, 197, 94, 0.1);
  border-left: 5px solid #22c55e;
  border-radius: 8px;
  padding: 1.5rem;
  display: flex;
  gap: 1.5rem;
  margin-top: 2rem;
}

.box-icon {
  font-size: 2.5rem;
}
.box-title {
  color: #86efac;
  font-weight: 700;
  font-size: 1.1rem;
  margin-bottom: 0.5rem;
}
.box-content p {
  color: #d1fae5;
  font-size: 0.95rem;
  margin: 0;
}

/* MATH DEEP DIVE */
.math-deep-dive {
  background: #0f172a;
  border: 1px solid #334155;
  border-radius: 16px;
  padding: 2rem;
}

.math-header {
  font-size: 1.5rem;
  color: #f1f5f9;
  font-weight: 700;
  margin-bottom: 2rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
}

.matrix-viz-container {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 3rem;
  align-items: center;
}

.equation-block {
  font-family: 'Times New Roman', serif;
  font-size: 1.5rem;
  color: #bae6fd;
  text-align: center;
  padding: 2rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 12px;
}

.matrix-explanation ul {
  list-style: none;
  padding: 0;
}

.matrix-explanation li {
  margin-bottom: 1rem;
  padding-left: 1rem;
  border-left: 3px solid #64748b;
  color: #cbd5e1;
}

.matrix-explanation strong {
  color: #f1f5f9;
}

/* MATH MATRIX CSS - HTML FALLBACK */
.equation-container {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 1rem;
  background: rgba(0, 0, 0, 0.3);
  padding: 2rem;
  border-radius: 12px;
}

.eq-var,
.eq-op {
  font-family: 'Times New Roman', serif;
  font-size: 2rem;
  font-style: italic;
  color: #fff;
}

.matrix-wrapper {
  position: relative;
  padding: 0 10px;
}

.matrix-grid-3x3 {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 0.5rem 1.5rem;
  text-align: center;
}

.m-cell {
  font-family: 'Times New Roman', serif;
  color: #cbd5e1;
  font-size: 1.2rem;
}

.m-cell.diagonal {
  color: #60a5fa; /* Highlight diagonal */
  font-weight: 700;
}

.bracket {
  position: absolute;
  top: -5px;
  bottom: -5px;
  width: 15px;
  border: 2px solid #fff;
}

.l-bracket {
  left: 0;
  border-right: none;
  border-top-left-radius: 8px;
  border-bottom-left-radius: 8px;
}

.r-bracket {
  right: 0;
  border-left: none;
  border-top-right-radius: 8px;
  border-bottom-right-radius: 8px;
}

/* MATH MATRIX CSS - HTML FALLBACK */
.equation-container {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 1rem;
  background: rgba(0, 0, 0, 0.3);
  padding: 2rem;
  border-radius: 12px;
}

.eq-var,
.eq-op {
  font-family: 'Times New Roman', serif;
  font-size: 2rem;
  font-style: italic;
  color: #fff;
}

.matrix-wrapper {
  position: relative;
  padding: 0 10px;
}

.matrix-grid-3x3 {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 0.5rem 1.5rem;
  text-align: center;
}

.m-cell {
  font-family: 'Times New Roman', serif;
  color: #cbd5e1;
  font-size: 1.2rem;
}

.m-cell.diagonal {
  color: #60a5fa; /* Highlight diagonal */
  font-weight: 700;
}

.bracket {
  position: absolute;
  top: -5px;
  bottom: -5px;
  width: 15px;
  border: 2px solid #fff;
}

.l-bracket {
  left: 0;
  border-right: none;
  border-top-left-radius: 8px;
  border-bottom-left-radius: 8px;
}

.r-bracket {
  right: 0;
  border-left: none;
  border-top-right-radius: 8px;
  border-bottom-right-radius: 8px;
}

/* INTERACTIVE CALCULATOR */
.interactive-tool {
  background: rgba(30, 41, 59, 0.5);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 2rem;
}

.tool-title {
  font-size: 1.2rem;
  font-weight: 700;
  color: #fbbf24;
  margin-bottom: 0.5rem;
}
.tool-desc {
  color: #94a3b8;
  font-size: 0.9rem;
  margin-bottom: 1.5rem;
}

.calculator-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 2rem;
}

.input-col {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.result-col {
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  background: rgba(251, 191, 36, 0.1);
  border-radius: 12px;
  padding: 2rem;
  border: 1px dashed #fbbf24;
}

.result-label {
  color: #fcd34d;
  font-weight: 700;
}
.result-value {
  font-size: 2.5rem;
  font-weight: 800;
  color: #fff;
  font-family: monospace;
}
.result-unit {
  color: #94a3b8;
}

.viz-feedback {
  height: 100px;
  background: #1e293b;
  border-radius: 8px;
  margin-top: 1.5rem;
  position: relative;
  overflow: hidden;
}

.viz-object.box-cm {
  width: 40px;
  height: 40px;
  background: #fbbf24;
  position: absolute;
  top: 30px;
  left: 30px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 700;
  color: #000;
  font-size: 0.7rem;
  border-radius: 4px;
}

.viz-axis {
  width: 2px;
  height: 100%;
  background: #fff;
  position: absolute;
  top: 0;
  border-left: 1px dashed #fff;
  color: #fff;
  padding-left: 5px;
  font-size: 0.7rem;
}

.viz-line {
  height: 2px;
  background: #ef4444;
  position: absolute;
  top: 50px;
  left: 70px;
}

/* CONCEPT VIZ 4x4 */
.matrix-4x4-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  grid-template-rows: 1fr 1fr;
  gap: 10px;
  max-width: 400px;
  margin: 0 auto;
}

.m-quadrant {
  padding: 1.5rem;
  border-radius: 8px;
  text-align: center;
  transition: transform 0.2s;
}

.m-quadrant:hover {
  transform: scale(1.05);
}

.m-quadrant.rotation {
  background: rgba(239, 68, 68, 0.2);
  border: 1px solid #ef4444;
}
.m-quadrant.position {
  background: rgba(59, 130, 246, 0.2);
  border: 1px solid #3b82f6;
}
.m-quadrant.perspective {
  background: rgba(100, 116, 139, 0.2);
  border: 1px solid #64748b;
  opacity: 0.5;
}
.m-quadrant.scale {
  background: rgba(16, 185, 129, 0.2);
  border: 1px solid #10b981;
}

.q-label {
  font-weight: 700;
  color: #fff;
  margin-bottom: 0.5rem;
}
.q-desc {
  font-size: 0.75rem;
  color: #cbd5e1;
}

/* DH VIZ */
.dh-viz {
  display: flex;
  gap: 3rem;
  align-items: center;
  justify-content: center;
}

.dh-diagram {
  width: 300px;
  height: 300px;
  background: #0f172a;
  border: 1px solid #334155;
  border-radius: 50%;
  position: relative;
  display: flex;
  align-items: center;
  justify-content: center;
}

.link-segment {
  position: absolute;
  transform-origin: center left;
  transition: transform 0.3s ease-out;
}

.link-body {
  height: 10px;
  background: #c084fc;
  border-radius: 5px;
}

.joint-axis {
  width: 20px;
  height: 20px;
  border-radius: 50%;
  background: #fbbf24;
  position: absolute;
  left: -10px;
  top: -5px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 0.6rem;
  color: black;
  font-weight: 700;
}

.dh-controls {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
  min-width: 250px;
}

.control-row label {
  font-size: 0.85rem;
  color: #cbd5e1;
  margin-bottom: 0.25rem;
  display: block;
}

/* COMPLETION MSG */
.completion-msg {
  padding: 2rem;
  text-align: center;
  color: #4ade80;
  font-weight: 700;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 1rem;
  background: rgba(34, 197, 94, 0.1);
  border-radius: 8px;
}

@media (max-width: 1024px) {
  .hero-section {
    flex-direction: column;
    text-align: center;
  }
  .matrix-viz-container {
    grid-template-columns: 1fr;
  }
  .calculator-grid {
    grid-template-columns: 1fr;
  }
  .dh-viz {
    flex-direction: column;
  }
}
/* BLOCK B STYLES */

/* URDF ANATOMY */
.urdf-anatomy-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 1.5rem;
}

.anatomy-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  transition: transform 0.2s;
}

.anatomy-card:hover {
  transform: translateY(-5px);
}

.anatomy-card.visual {
  border-top: 4px solid #3b82f6;
}
.anatomy-card.collision {
  border-top: 4px solid #ef4444;
}
.anatomy-card.inertial {
  border-top: 4px solid #8b5cf6;
}

.card-header {
  padding: 1rem;
  background: rgba(0, 0, 0, 0.2);
  font-weight: 700;
  color: #f1f5f9;
  display: flex;
  align-items: center;
  gap: 0.5rem;
  font-family: monospace;
}

.card-body {
  padding: 1.5rem;
  font-size: 0.9rem;
  color: #cbd5e1;
}

.card-body ul {
  margin: 0;
  padding-left: 1.2rem;
}
.card-body li {
  margin-bottom: 0.5rem;
}

.warning-box {
  margin-top: 1rem;
  background: rgba(239, 68, 68, 0.1);
  border: 1px dashed #ef4444;
  padding: 0.75rem;
  border-radius: 6px;
  font-size: 0.8rem;
  color: #fca5a5;
}

/* XACRO STUDIO */
.xacro-studio {
  background: #1e293b;
  border-radius: 12px;
  border: 1px solid #334155;
  overflow: hidden;
}

.studio-header {
  padding: 0.75rem 1rem;
  background: #334155;
  color: #fff;
  font-weight: 700;
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.split-view {
  display: grid;
  grid-template-columns: 1fr auto 1fr;
  align-items: center;
  padding: 1rem;
  gap: 1rem;
}

.code-pane {
  background: #0f172a;
  border-radius: 8px;
  height: 350px;
  display: flex;
  flex-direction: column;
}

.pane-label {
  font-size: 0.75rem;
  color: #94a3b8;
  padding: 0.5rem;
  border-bottom: 1px solid #334155;
  font-family: monospace;
}

.arrow-pane {
  display: flex;
  flex-direction: column;
  align-items: center;
  color: #64748b;
  font-size: 0.8rem;
}

.arrow-pane .q-icon {
  font-size: 2rem;
  color: #fbbf24;
}

.xml-tree {
  padding: 1rem;
  font-family: monospace;
  font-size: 0.8rem;
  color: #a5f3fc;
  overflow-y: auto;
}

.indent {
  margin-left: 1rem;
}
.indent-2 {
  margin-left: 2rem;
  color: #64748b;
}

/* COLLISION VIZ */
.collision-viz-container {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 1.5rem;
}

.col-card {
  background: rgba(15, 23, 42, 0.6);
  border-radius: 8px;
  padding: 1rem;
  text-align: center;
}

.col-card.bad {
  border: 1px solid #ef4444;
}
.col-card.ok {
  border: 1px solid #f59e0b;
}
.col-card.good {
  border: 1px solid #22c55e;
}

.col-title {
  font-weight: 700;
  margin-bottom: 1rem;
  color: #fff;
}

.col-viz {
  height: 120px;
  display: flex;
  align-items: center;
  justify-content: center;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
  margin-bottom: 1rem;
}

.col-svg {
  width: 80px;
  height: 80px;
}
.primitive-box {
  width: 60px;
  height: 60px;
  background: #22c55e;
  border: 2px solid #fff;
}

.col-stats {
  font-size: 0.8rem;
  color: #94a3b8;
  font-family: monospace;
}
/* BLOCK C STYLES */

/* PIPELINE VIZ */
.pipeline-viz {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 1rem;
  background: rgba(15, 23, 42, 0.6);
  padding: 2rem;
  border-radius: 12px;
  flex-wrap: wrap;
}

.p-step {
  display: flex;
  flex-direction: column;
  align-items: center;
  width: 200px;
  text-align: center;
  background: #1e293b;
  padding: 1.5rem;
  border-radius: 8px;
  border: 1px solid #334155;
}

.p-step.cad {
  border-top: 4px solid #ef4444;
}
.p-step.blender {
  border-top: 4px solid #f97316;
}
.p-step.sim {
  border-top: 4px solid #3b82f6;
}

.p-icon {
  font-size: 2.5rem;
  color: #fff;
  margin-bottom: 0.5rem;
}
.p-label {
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.5rem;
}
.p-desc {
  font-size: 0.8rem;
  color: #94a3b8;
}
.p-arrow {
  font-size: 1.5rem;
  color: #64748b;
  font-weight: 700;
}

/* MESH LAB */
.mesh-lab-container {
  background: #0f172a;
  border: 1px dashed #64748b;
  border-radius: 12px;
  padding: 2rem;
  position: relative;
}

.lab-title {
  position: absolute;
  top: -15px;
  left: 20px;
  background: #0f172a;
  padding: 0 10px;
  color: #c4b5fd;
  font-weight: 700;
  font-family: monospace;
}

.errors-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 1.5rem;
}

.error-card {
  background: rgba(30, 41, 59, 0.5);
  padding: 1rem;
  border-radius: 8px;
}

.e-header {
  color: #fca5a5;
  font-weight: 700;
  margin-bottom: 1rem;
  text-align: center;
}

.e-viz {
  height: 100px;
  background: rgba(0, 0, 0, 0.3);
  margin-bottom: 1rem;
  display: flex;
  align-items: center;
  justify-content: center;
  position: relative;
}

/* Specific Viz Logic (Simplified CSS) */
.invert .face {
  width: 50px;
  height: 50px;
  position: absolute;
}
.invert .face.front {
  border: 2px solid green;
  z-index: 1;
  display: none;
} /* Missing face */
.invert .face.back {
  border: 2px dashed red;
  background: rgba(255, 0, 0, 0.1);
}

.origin .origin-point {
  width: 4px;
  height: 4px;
  background: cyan;
  position: absolute;
  top: 50%;
  left: 50%;
  box-shadow: 0 0 10px cyan;
}
.origin .ghost-mesh {
  border: 1px dashed white;
  width: 40px;
  height: 40px;
  position: absolute;
  top: 10%;
  right: 10%;
  color: white;
  font-size: 0.7rem;
  display: flex;
  align-items: center;
  justify-content: center;
}

.scale .measure {
  font-size: 0.7rem;
  font-family: monospace;
}
.scale .big {
  font-size: 1.2rem;
  color: #ef4444;
}
.scale .small {
  font-size: 0.6rem;
  color: #94a3b8;
}

.e-fix {
  font-size: 0.8rem;
  color: #cbd5e1;
  line-height: 1.4;
}

/* TRANSMISSION ARCH */
.transmission-architecture {
  background: #1e293b;
  padding: 2rem;
  border-radius: 12px;
}

.arch-diagram {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
  align-items: center;
  margin-bottom: 2rem;
}

.arch-block {
  width: 300px;
  background: #334155;
  border: 1px solid #475569;
  border-radius: 6px;
  padding: 1rem;
  text-align: center;
  position: relative;
}

.arch-block:after {
  content: '⬇';
  position: absolute;
  bottom: -20px;
  left: 50%;
  transform: translateX(-50%);
  color: #64748b;
}
.arch-block:last-child:after {
  content: none;
}

.arch-block.controller {
  border-left: 4px solid #22c55e;
}
.arch-block.interface {
  border-left: 4px solid #f59e0b;
}
.arch-block.transmission {
  border-left: 4px solid #3b82f6;
}
.arch-block.joint {
  border-left: 4px solid #a855f7;
}

.a-title {
  font-weight: 700;
  color: #fff;
}
.a-sub {
  font-size: 0.8rem;
  color: #94a3b8;
}
.mini-code {
  font-size: 0.7rem;
  text-align: left;
  margin-top: 0.5rem;
}

@media (max-width: 1024px) {
  .pipeline-viz,
  .errors-grid {
    grid-template-columns: 1fr;
  }
}
/* BLOCK D STYLES */

/* BLUEPRINT VIZ */
.blueprint-container {
  background: #0f172a;
  border: 2px solid #3b82f6;
  border-radius: 4px;
  padding: 1rem;
  font-family: monospace;
  position: relative;
  background-image:
    linear-gradient(rgba(59, 130, 246, 0.1) 1px, transparent 1px),
    linear-gradient(90deg, rgba(59, 130, 246, 0.1) 1px, transparent 1px);
  background-size: 20px 20px;
}

.bp-header {
  border-bottom: 2px solid #3b82f6;
  padding-bottom: 0.5rem;
  margin-bottom: 1.5rem;
}

.bp-title {
  font-size: 1.5rem;
  font-weight: 700;
  color: #60a5fa;
  letter-spacing: 2px;
}

.bp-meta {
  color: #3b82f6;
  font-size: 0.7rem;
}

.bp-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 2rem;
}

.bp-section {
  border: 1px dashed #1e40af;
  padding: 1rem;
  background: rgba(15, 23, 42, 0.8);
}

.robot-schematic {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 5px;
}

.r-base {
  padding: 10px 20px;
  background: #334155;
  color: #fff;
  border: 1px solid #fff;
}
.r-joint {
  width: 20px;
  height: 20px;
  border-radius: 50%;
  background: #fbbf24;
  color: #000;
  font-size: 0.6rem;
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 2;
}
.r-link {
  width: 10px;
  height: 40px;
  background: #60a5fa;
  border: 1px solid #fff;
}
.r-ee {
  padding: 5px;
  border: 2px solid #ef4444;
  color: #ef4444;
  font-size: 0.7rem;
}

.map-node {
  padding: 0.5rem;
  color: #cbd5e1;
  border-left: 2px solid #334155;
  margin-bottom: 0.5rem;
}

.map-node.root {
  color: #fbbf24;
  font-weight: 700;
  border: none;
}
.map-node.indented {
  margin-left: 2rem;
  border-left: 1px dashed #64748b;
  padding-left: 0.5rem;
}

.node-desc {
  font-size: 0.7rem;
  color: #64748b;
  margin-left: 1.5rem;
}

/* CHALLENGE VIZ */
.challenge-container {
  background: rgba(239, 68, 68, 0.1);
  border: 1px solid #ef4444;
  border-radius: 12px;
  padding: 2rem;
}

.challenge-header {
  display: flex;
  gap: 1.5rem;
  margin-bottom: 2rem;
}

.c-icon {
  font-size: 3rem;
  animation: pulse 2s infinite;
}

.c-title {
  font-size: 1.2rem;
  font-weight: 700;
  color: #fca5a5;
  margin-bottom: 0.5rem;
}
.c-desc {
  color: #fecaca;
  font-size: 0.95rem;
}

.challenge-options {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.c-option {
  display: flex;
  align-items: flex-start; /* Changed from center for multi-line text */
  gap: 1rem;
  padding: 1rem;
  background: rgba(0, 0, 0, 0.2);
  border-radius: 8px;
  cursor: pointer;
  transition: all 0.2s;
  flex-wrap: wrap; /* Allow wrapping */
}

.c-option:hover {
  background: rgba(255, 255, 255, 0.05);
}

.opt-radio {
  width: 30px;
  height: 30px;
  border-radius: 50%;
  border: 2px solid #cbd5e1;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 700;
  color: #cbd5e1;
  flex-shrink: 0; /* Prevent shrinking */
}

.opt-text {
  color: #fff;
  flex: 1;
}

.c-option.correct {
  border: 1px solid #22c55e;
  background: rgba(34, 197, 94, 0.1);
}
.c-option.correct .opt-radio {
  border-color: #22c55e;
  color: #22c55e;
}
.opt-feedback {
  width: 100%;
  margin-top: 0.5rem;
  font-size: 0.9rem;
  color: #86efac;
  padding-left: 3.5rem;
}

/* SUMMARY SECTION */
.summary-section {
  margin-top: 5rem;
  background: #1e293b;
  padding: 3rem;
  border-radius: 20px;
  text-align: center;
}

.summary-title {
  font-size: 2rem;
  font-weight: 800;
  color: #fff;
  margin-bottom: 2rem;
}

.summary-list {
  list-style: none;
  padding: 0;
  max-width: 600px;
  margin: 0 auto 3rem auto;
  text-align: left;
}

.summary-list li {
  display: flex;
  align-items: center;
  gap: 1rem;
  margin-bottom: 1rem;
  font-size: 1.1rem;
  color: #cbd5e1;
}

.summary-list .q-icon {
  color: #22c55e;
  font-size: 1.5rem;
}

.next-step {
  background: rgba(59, 130, 246, 0.1);
  padding: 2rem;
  border-radius: 12px;
  display: inline-block;
  border: 1px solid #3b82f6;
}

.next-label {
  color: #94a3b8;
  font-size: 0.9rem;
  text-transform: uppercase;
  letter-spacing: 1px;
  margin-bottom: 0.5rem;
}
.next-title {
  color: #fff;
  font-size: 1.3rem;
  font-weight: 700;
  margin-bottom: 1rem;
}

@media (max-width: 1024px) {
  .bp-grid {
    grid-template-columns: 1fr;
  }
}

@keyframes pulse {
  0% {
    transform: scale(1);
  }
  50% {
    transform: scale(1.1);
  }
  100% {
    transform: scale(1);
  }
}
/* VIDEO TUTORIAL STYLES */
.video-container {
  background: #0f172a;
  border: 1px solid #334155;
  border-radius: 12px;
  overflow: hidden;
  padding: 1rem;
  box-shadow: 0 10px 30px -5px rgba(0, 0, 0, 0.5);
}

.video-wrapper {
  position: relative;
  padding-bottom: 56.25%; /* 16:9 Aspect Ratio */
  height: 0;
  overflow: hidden;
  border-radius: 8px;
  background: #000;
  box-shadow: inset 0 0 20px rgba(0, 0, 0, 0.5);
}

.video-wrapper iframe {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
}

.video-caption {
  padding: 1rem 0.5rem 0.5rem 0.5rem;
  color: #94a3b8;
  font-size: 0.9rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
}

.video-caption .q-icon {
  font-size: 1.5rem;
  color: #ef4444; /* YouTube Red */
}

/* SUMMARY STYLES (Refactored) */
.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
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
  color: #14b8a6;
  font-size: 0.95rem;
  font-weight: 700;
}

.summary-item span {
  color: #cbd5e1;
  font-size: 0.85rem;
}

.next-step {
  background: rgba(59, 130, 246, 0.1);
  padding: 2rem;
  border-radius: 12px;
  display: inline-block;
  border: 1px solid #3b82f6;
  text-align: center;
  width: 100%;
}

.next-label {
  color: #94a3b8;
  font-size: 0.9rem;
  text-transform: uppercase;
  letter-spacing: 1px;
  margin-bottom: 0.5rem;
}

.next-title {
  color: #fff;
  font-size: 1.3rem;
  font-weight: 700;
  margin-bottom: 1rem;
}
</style>
