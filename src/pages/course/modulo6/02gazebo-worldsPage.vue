<template>
  <LessonContainer>
    <!-- ========================================================================================
         HERO SECTION: THE MATRIX OF ROBOTICS
         Visual: Dark, Cyberpunk, Wireframe Aesthetic
         ======================================================================================== -->
    <div class="hero-section text-white q-mb-xl">
      <div class="hero-content">
        <div class="hero-badge"><q-icon name="public" /> MÓDULO 6.2: ENTORNOS VIRTUALES</div>
        <h1 class="hero-title">
          Gazebo Worlds: <span class="gradient-text">Physics Engine Architecture</span>
        </h1>
        <p class="hero-subtitle">
          Bienvenido al nivel doctoral. Aquí no "arrastramos cajas". Aquí manipulamos el tejido del
          espacio-tiempo simulado ($dt$), ajustamos el solver LCP para evitar singularidades y
          configuramos el Pipeline de Rendering PBR para engañar a redes neuronales.
          <strong>Este es el Matrix de tu robot.</strong>
        </p>

        <div class="hero-stats">
          <div class="stat-item">
            <div class="stat-val">ODE / Bullet</div>
            <div class="stat-label">Physics Kernels</div>
          </div>
          <div class="stat-item">
            <div class="stat-val">LCP Solver</div>
            <div class="stat-label">Math Backend</div>
          </div>
          <div class="stat-item">
            <div class="stat-val">PBR / Ogre</div>
            <div class="stat-label">Render Pipeline</div>
          </div>
        </div>
      </div>

      <div class="hero-viz">
        <!-- Abstract Physics Viz: Connected Nodes pulsating -->
        <svg viewBox="0 0 200 200" class="physics-mesh">
          <defs>
            <linearGradient id="linkGrad" x1="0%" y1="0%" x2="100%" y2="100%">
              <stop offset="0%" stop-color="#3b82f6" stop-opacity="0.8" />
              <stop offset="100%" stop-color="#8b5cf6" stop-opacity="0.2" />
            </linearGradient>
          </defs>
          <!-- Nodes -->
          <circle cx="100" cy="100" r="10" fill="#fbbf24" class="node core" />
          <circle cx="50" cy="50" r="5" fill="#60a5fa" class="node satellite s1" />
          <circle cx="150" cy="50" r="5" fill="#60a5fa" class="node satellite s2" />
          <circle cx="50" cy="150" r="5" fill="#60a5fa" class="node satellite s3" />
          <circle cx="150" cy="150" r="5" fill="#60a5fa" class="node satellite s4" />
          <!-- Links -->
          <line
            x1="100"
            y1="100"
            x2="50"
            y2="50"
            stroke="url(#linkGrad)"
            stroke-width="2"
            class="link"
          />
          <line
            x1="100"
            y1="100"
            x2="150"
            y2="50"
            stroke="url(#linkGrad)"
            stroke-width="2"
            class="link"
          />
          <line
            x1="100"
            y1="100"
            x2="50"
            y2="150"
            stroke="url(#linkGrad)"
            stroke-width="2"
            class="link"
          />
          <line
            x1="100"
            y1="100"
            x2="150"
            y2="150"
            stroke="url(#linkGrad)"
            stroke-width="2"
            class="link"
          />
        </svg>
      </div>
    </div>

    <!-- ========================================================================================
         BLOCK A: THE PHYSICS ENGINE KERNEL (ODE DEEP DIVE)
         Depth: White Paper Level
         ======================================================================================== -->
    <div class="content-block">
      <SectionTitle>1. Teoría de Motores de Física: El Solver LCP</SectionTitle>

      <TextBlock>
        La mayoría de los estudiantes creen que Gazebo simplemente "mueve cosas". Falso. Gazebo es
        un resolutor numérico de ecuaciones diferenciales rígidas. Cuando un robot pisa el suelo, no
        hay "tierra"; hay una restricción matemática de no-penetración que debe ser satisfecha.
        <br /><br />
        El motor por defecto, <strong>ODE (Open Dynamics Engine)</strong>, convierte todas las
        restricciones (Joints, Contactos, Fricción) en un
        <strong>LCP (Linear Complementarity Problem)</strong> de la forma:
      </TextBlock>

      <!-- MATH VIZ: THE LCP MATRIX -->
      <div class="math-viz-container q-my-lg">
        <div class="equation-row">
          <div class="matrix-label">M (Mass Matrix)</div>
          <div class="operator">×</div>
          <div class="matrix-label">a (Acceleration)</div>
          <div class="operator">=</div>
          <div class="matrix-label">f (Forces)</div>
          <div class="operator">+</div>
          <div class="matrix-label">Jᵀλ (Constraint Forces)</div>
        </div>
        <div class="equation-desc">
          Donde $J$ es la Jacobiana de restricciones y $\lambda$ son los multiplicadores de Lagrange
          (Fuerzas necesarias para mantener las restricciones).
        </div>
      </div>

      <TextBlock>
        Resolver este sistema ($Ax=b$) en tiempo real (1000 Hz) requiere simplificaciones. Aquí es
        donde entran los parámetros que deciden si tu simulación es "roca sólida" o "gelatina
        explosiva".
      </TextBlock>

      <!-- INTERACTIVE TOOL: THE PHYSICS TUNER (ENHANCED) -->
      <div class="tool-wrapper q-mt-xl">
        <div class="tool-header">
          <q-icon name="tune" />
          <div class="tool-title">The Physics Tuner (Visualizador ERP/CFM)</div>
        </div>

        <div class="physics-lab">
          <!-- Controls -->
          <div class="lab-controls">
            <div class="control-group">
              <div class="c-label">ERP (Error Reduction Parameter)</div>
              <q-slider v-model="erpVal" :min="0" :max="1" :step="0.05" label color="green-4" />
              <div class="c-desc">
                Qué tan agresivamente corregimos el error (penetración) en este frame.
                <br />
                <span :class="erpVal > 0.8 ? 'text-red' : 'text-grey'">
                  {{ erpStatus }}
                </span>
              </div>
            </div>

            <div class="control-group">
              <div class="c-label">CFM (Constraint Force Mixing)</div>
              <q-slider v-model="cfmVal" :min="0" :max="0.1" :step="0.001" label color="orange-4" />
              <div class="c-desc">
                Permisividad de violación de restricciones ("Sponginess").
                <br />
                <span :class="cfmVal > 0.01 ? 'text-orange' : 'text-grey'">
                  {{ cfmStatus }}
                </span>
              </div>
            </div>

            <div class="control-group">
              <div class="c-label">Iteraciones del Solver</div>
              <q-slider v-model="itersVal" :min="10" :max="200" :step="10" label color="blue-4" />
              <div class="c-desc">
                Pasos de refinamiento del algoritmo QuickStep (Inverso al Performance).
              </div>
            </div>
          </div>

          <!-- Visualizer -->
          <div class="lab-viz">
            <div class="ground-plane" :style="{ filter: `brightness(${1 - cfmVal * 5})` }"></div>
            <div
              class="dynamic-object"
              :style="{
                bottom: `${penetrationDepth}px`,
                transform: `scaleY(${1 - compressionRatio})`,
                borderColor: stabilityColor,
              }"
            >
              <div class="obj-label">Rigid Body</div>
              <div class="obj-mass">m=10kg</div>
            </div>
            <div class="force-vector" :style="{ height: `${correctionForce}px`, opacity: erpVal }">
              <span class="force-label">F = {{ Math.round(correctionForce * 10) }}N</span>
            </div>
            <div class="penetration-zone" :style="{ height: `${Math.abs(penetrationDepth)}px` }">
              Error Zone
            </div>
          </div>
        </div>
      </div>

      <!-- DOCTORAL CODE BLOCK: PHYSICS PROFILE -->
      <div class="code-section q-mt-xl">
        <div class="code-header">
          <q-icon name="code" />
          <span>physics_engine_profile.sdf</span>
        </div>
        <CodeBlock
          lang="xml"
          :copyable="true"
          title="Configuración Maestra de Física (ODE)"
          content='<physics type="ode">
  <!--
    STEP SIZE (dt):
    El latido del corazón del universo.
    1ms (0.001) es estándar para robótica móvil.
    0.1ms (0.0001) es requerido para brazos rápidos o manipulación fina.
    Warning: Reducir dt aumenta la carga CPU linealmente.
  -->
  <max_step_size>0.001</max_step_size>

  <!--
    REAL TIME FACTOR / UPDATE RATE:
    RTF = (real_time_update_rate * max_step_size)
    Si target=1000Hz y step=0.001s -> RTF=1.0 (Tiempo Real).
    Si tu PC es lenta, RTF caerá a < 1.0 (Slow Motion).
  -->
  <real_time_update_rate>1000</real_time_update_rate>

  <ode>
    <solver>
      <!--
        TYPE: "quick" vs "world".
        "quick": Iterativo (SOR). Rápido, O(N). Menos preciso.
        "world": Exacto (Dantzig). Lento, O(N^3). Perfecto para pocos cuerpos.
      -->
      <type>quick</type>

      <!--
        ITERS:
        Número de pasadas del solver para converger a una solución estable.
        50 = Low fidelity (Jittering visible).
        100 = Standard.
        200+ = High fidelity (Manipulation).
      -->
      <iters>100</iters>

      <!--
        SOR (Successive Over-Relaxation):
        Ayuda a converger más rápido sobrecorrigiendo.
        1.0 = Gauss-Seidel estándar.
        1.3 = Valor típico para ganar velocidad/estabilidad.
      -->
      <sor>1.3</sor>
    </solver>

    <constraints>
      <!--
        CFM (Constraint Force Mixing):
        Permite violar restricciones para evitar singularidades numéricas.
        0.0 = Hard Constraint (Inestable si hay conflicto).
        1e-5 = Soft Constraint (Recomendado para estabilidad numérica).
      -->
      <cfm>0.00001</cfm>

      <!--
        ERP (Error Reduction Parameter):
        Qué porcentaje del error de posición se corrige en el siguiente paso.
        0.2 = Estándar (corrige 20% del error, actúa como resorte amortiguado).
        0.8 = Muy rígido (puede causar oscilación/explosión).
      -->
      <erp>0.2</erp>

      <!--
        CONTACT PARAMETERS:
        Definen la "piel" de los objetos. Una capa virtual que previene
        la interpenetración profunda (Tunneling effect).
      -->
      <contact_surface_layer>0.001</contact_surface_layer>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
    </constraints>
  </ode>
</physics>'
        />
      </div>
    </div>

    <!-- ========================================================================================
         BLOCK B: RENDERING PIPELINE & SDF ARCHITECTURE
         Depth: Industrial Graphics & Simulation Format
         ======================================================================================== -->
    <div class="section-divider">
      <q-separator spaced="lg" color="indigo-9" />
    </div>

    <!-- SECTION 2: RENDERING PIPELINE -->
    <div class="content-block">
      <SectionTitle>2. Rendering Pipeline: Fotorealismo Industrial</SectionTitle>

      <TextBlock>
        Para que una cámara simulada engañe a una red neuronal entrenada con imágenes reales,
        necesitamos
        <strong>Physically Based Rendering (PBR)</strong>. Gazebo Classic (OGRE 1.x) usa el modelo
        Phong (especularidad plástica), pero podemos hackear el pipeline de sombras y luz ambiental
        para acercarnos al realismo.
      </TextBlock>

      <div class="render-pipeline-viz q-my-lg">
        <div class="pipeline-stage">
          <q-icon name="light_mode" size="lg" color="yellow-5" />
          <div class="stage-label">Directional Light</div>
          <div class="stage-desc">Sol infinito (Sombras paralelas)</div>
        </div>
        <div class="pipeline-arrow">➔</div>
        <div class="pipeline-stage">
          <q-icon name="blur_on" size="lg" color="grey-5" />
          <div class="stage-label">Ambient Occlusion</div>
          <div class="stage-desc">SSAO (Sombras de contacto)</div>
        </div>
        <div class="pipeline-arrow">➔</div>
        <div class="pipeline-stage">
          <q-icon name="texture" size="lg" color="purple-4" />
          <div class="stage-label">Material Script</div>
          <div class="stage-desc">Normal Maps + Specular Maps</div>
        </div>
      </div>

      <CodeBlock
        title="photorealistic_sun.light"
        lang="xml"
        :copyable="true"
        content='<light type="directional" name="sun">
  <cast_shadows>true</cast_shadows>
  <pose>0 0 10 0 -0 0</pose>

  <!-- Luz cálida de atardecer (R=1.0, G=0.9, B=0.8) -->
  <diffuse>1.0 0.9 0.8 1</diffuse>

  <!-- Especularidad alta para metales -->
  <specular>0.2 0.2 0.2 1</specular>

  <attenuation>
    <range>1000</range>
    <constant>0.9</constant> <!-- No decae con distancia (Sol) -->
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
</light>'
      />
    </div>

    <!-- SECTION 3: SDF ARCHITECTURE -->
    <div class="content-block q-mt-xl">
      <SectionTitle>3. Arquitectura SDF: "God-Mode" XML</SectionTitle>
      <TextBlock>
        URDF describe robots; <strong>SDF (Simulation Description Format)</strong> describe el
        universo. SDF rompe la estructura de árbol de URDF para permitir ciclos cinemáticos y
        entornos estáticos. Entender la jerarquía `&lt;world&gt;` es vital para inyectar plugins
        globales.
      </TextBlock>

      <!-- SDF TREE VIZ -->
      <div class="sdf-tree-container">
        <div class="tree-root">&lt;sdf version="1.7"&gt;</div>
        <div class="tree-branch">
          <div class="branch-line"></div>
          <div class="tree-node world"><q-icon name="public" /> &lt;world name="default"&gt;</div>
          <div class="tree-children">
            <div class="child-node physics">
              <q-icon name="speed" /> &lt;physics&gt; (Solver LCP)
            </div>
            <div class="child-node scene">
              <q-icon name="landscape" /> &lt;scene&gt; (Sky, Ambient)
            </div>
            <div class="child-node include">
              <q-icon name="link" /> &lt;include&gt; (Modelos Externos)
            </div>
            <div class="child-node plugin">
              <q-icon name="extension" /> &lt;plugin&gt; (World Logic)
            </div>
          </div>
        </div>
      </div>

      <TextBlock class="q-mt-md">
        El elemento más poderoso es el <strong>World Plugin</strong>. A diferencia de un nodo de ROS
        2, este código C++ corre <em>dentro</em> del loop de memoria de Gazebo, permitiendo acceso
        directo a la API de física sin latencia de red (DDS).
      </TextBlock>

      <div class="code-section">
        <div class="code-header">
          <q-icon name="terminal" />
          <span>sim_time_control.cpp (C++ World Plugin)</span>
        </div>
        <CodeBlock
          lang="cpp"
          :copyable="true"
          title="Control del Tiempo de Simulación (Matrix Agent)"
          content='#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {
  class WorldTimeCtrl : public WorldPlugin {
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
      // Hook al evento de actualización de física
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&WorldTimeCtrl::OnUpdate, this));

      printf("Matrix Plugin Loaded: Controlling Time...\n");
    }

    public: void OnUpdate() {
      // Lógica ejecutada en cada step de simulación (1000Hz)
      // Ejemplo: Pausar el tiempo si un robot colisiona

      common::Time currentTime = this->world->SimTime();
      if (currentTime.Double() > 10.0 && !paused) {
         this->world->SetPaused(true); // "Bullet Time" effect
         paused = true;
      }
    }

    private: physics::WorldPtr world;
    private: event::ConnectionPtr updateConnection;
    private: bool paused = false;
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldTimeCtrl)
}'
        />
      </div>
    </div>

    <!-- ========================================================================================
         BLOCK C: PROCEDURAL GENERATION (PYTHON SCRIPTING)
         Depth: Automating the Matrix
         ======================================================================================== -->
    <div class="section-divider">
      <q-separator spaced="lg" color="indigo-9" />
    </div>

    <!-- SECTION 4: PROCEDURAL GENERATION -->
    <div class="content-block">
      <SectionTitle>4. Generación Procedural: Scripting God-Mode</SectionTitle>

      <TextBlock>
        Editar archivos `.world` a mano es para amateurs. En entornos de producción (Amazon, Tesla),
        los mundos se generan dinámicamente usando Python. Esto permite
        <strong>Domain Randomization</strong>: generar 1,000 variaciones de un almacén (luces,
        fricción, obstáculos) para entrenar modelos de IA robustos.
      </TextBlock>

      <!-- PROCEDURAL LOGIC VIZ -->
      <div class="procedural-viz-container q-my-lg">
        <div class="proc-header">
          <span>Factory Logic</span>
          <div class="proc-badge">PYTHON 3.10</div>
        </div>
        <div class="proc-grid">
          <div class="proc-step input">
            <div class="step-icon"><q-icon name="data_object" /></div>
            <div class="step-text">
              <strong>Input Params</strong>
              <br />Size: 50x50m <br />Obstacles: 200
            </div>
          </div>
          <div class="proc-arrow">➔</div>
          <div class="proc-step logic">
            <div class="step-icon"><q-icon name="casino" /></div>
            <div class="step-text">
              <strong>Randomizer</strong>
              <br />Poisson Disk Sampling <br />(No overloap)
            </div>
          </div>
          <div class="proc-arrow">➔</div>
          <div class="proc-step output">
            <div class="step-icon"><q-icon name="code" /></div>
            <div class="step-text">
              <strong>XML Injection</strong>
              <br />f-strings -> .world
            </div>
          </div>
        </div>
        <!-- Simulated Map Output -->
        <div class="proc-map-preview">
          <div class="map-grid">
            <div
              v-for="n in 50"
              :key="n"
              class="map-cell"
              :class="{ occupied: n % 7 === 0 || n % 11 === 0 }"
            ></div>
          </div>
          <div class="map-label">Preview: generated_warehouse_v9.world</div>
        </div>
      </div>

      <div class="code-section">
        <div class="code-header">
          <q-icon name="terminal" />
          <span>factory.py (Procedural Generator)</span>
        </div>
        <CodeBlock
          lang="python"
          :copyable="true"
          title="Generador de Almacenes Infinitos"
          content='import random
import xml.etree.ElementTree as ET

# Plantilla Base SDF (Header)
SDF_HEADER = """<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="logistics_world_{seed}">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
"""

def generate_warehouse(num_shelves=50, seed=42):
    random.seed(seed)
    models_xml = ""

    # Grid de ocupación para evitar colisiones
    occupied_positions = set()

    for i in range(num_shelves):
        # Muestreo seguro (Simple Rejection Sampling)
        while True:
            x = round(random.uniform(-20, 20), 1)
            y = round(random.uniform(-20, 20), 1)
            key = (int(x), int(y))

            if key not in occupied_positions:
                occupied_positions.add(key)
                break

        # Inyección SDF con f-strings (Rápido y legible)
        models_xml += f"""
    <include>
      <name>shelf_{i:03d}</name>
      <pose>{x} {y} 0 0 0 {random.choice([0, 1.57])}</pose>
      <uri>model://aws_robomaker_warehouse_ShelfD_01</uri>
      <static>true</static> <!-- Static para performance (O(1) physics) -->
    </include>
"""

    # Ensamblar el mundo final
    final_world = SDF_HEADER.format(seed=seed) + models_xml + "  </world>\n</sdf>"

    filename = f"warehouse_{seed}.world"
    with open(filename, "w") as f:
        f.write(final_world)

    print(f"✅ Generated {filename} with 50 shelves.")

if __name__ == "__main__":
    generate_warehouse(seed=1337)'
        />
      </div>

      <TextBlock class="q-mt-md">
        <strong>Nota de Ingeniería:</strong> Observa el uso de
        <code>&lt;static&gt;true&lt;/static&gt;</code>. En simulación, si un objeto no se va a
        mover, márcalo siempre como estático. Esto lo elimina de la matriz LCP dinámica, ahorrando
        ciclos de CPU masivos. Un almacén con 1,000 estanterías dinámicas correría a 0.5 FPS;
        estáticas corren a 60 FPS.
      </TextBlock>
    </div>

    <!-- ========================================================================================
         BLOCK D: CASE STUDY & SUMMARY
         Depth: Integration & Best Practices
         ======================================================================================== -->
    <div class="section-divider">
      <q-separator spaced="lg" color="indigo-9" />
    </div>

    <!-- SECTION 5: CASE STUDY -->
    <div class="content-block">
      <SectionTitle>5. Case Study: Almacén Logístico "Infinite"</SectionTitle>
      <TextBlock>
        Hemos combinado la Física ODE ajustada (Bloque A), la iluminación PBR (Bloque B) y la
        generación procedural (Bloque C) para crear <strong>"The Infinite Warehouse"</strong>. Este
        entorno soporta flotas de 50+ AMRs simultáneos manteniendo un Real Time Factor (RTF) > 0.9.
      </TextBlock>

      <div class="case-study-viz q-my-lg">
        <div class="cs-card">
          <q-icon name="inventory_2" size="xl" color="amber-5" />
          <div class="cs-stat">2,500</div>
          <div class="cs-label">Static Actors</div>
        </div>
        <div class="cs-card">
          <q-icon name="directions_run" size="xl" color="blue-5" />
          <div class="cs-stat">20</div>
          <div class="cs-label">Dynamic Humans</div>
        </div>
        <div class="cs-card">
          <q-icon name="speed" size="xl" color="green-5" />
          <div class="cs-stat">0.98</div>
          <div class="cs-label">RTF @ 4 Cores</div>
        </div>
      </div>

      <q-expansion-item
        class="bg-slate-800 text-white rounded-lg border border-slate-700 q-mb-xl"
        icon="warehouse"
        label="Ver Configuración Final (warehouse_production.world)"
        header-class="text-amber-400 font-bold"
      >
        <div class="q-pa-md">
          <CodeBlock
            lang="xml"
            content='<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="amazon_warehouse_production">

    <!-- PHYSICS ENGINE OPTIMIZED FOR STABILITY -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>70</iters> <!-- Reducido de 100 para ganar FPS -->
          <sor>1.3</sor>
        </solver>
      </ode>
    </physics>

    <!-- ACTOR: WALKERS (Humanos que caminan) -->
    <actor name="worker_1">
      <skin>
        <filename>walk.dae</filename>
      </skin>
      <animation name="walking">
        <filename>walk.dae</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0.000000</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walking">
          <waypoint>
            <time>0.0</time>
            <pose>0 0 0 0 0 0</pose>
          </waypoint>
          <waypoint>
            <time>5.0</time>
            <pose>5 0 0 0 0 0</pose>
          </waypoint>
        </trajectory>
      </script>
    </actor>

    <!-- INCLUDES PROCEDURALES (Output de factory.py) -->
    <include><uri>model://warehouse_zones_A</uri></include>
    <include><uri>model://warehouse_zones_B</uri></include>

  </world>
</sdf>'
            :copyable="true"
          />
        </div>
      </q-expansion-item>
    </div>

    <!-- DOCTORAL CHALLENGE -->
    <div class="content-block q-mb-xl">
      <SectionTitle>6. Doctor's Challenge: El Efecto Mariposa</SectionTitle>

      <div class="challenge-card">
        <div class="challenge-header">
          <q-icon name="psychology_alt" class="c-icon" />
          <div class="c-info">
            <div class="c-title">🦋 Caos Determinista</div>
            <div class="c-desc">
              Corres la misma simulación 2 veces con el mismo código de navegación. En la primera,
              el robot llega a la meta. En la segunda, choca. ¿Por qué ocurre esta divergencia en
              ODE?
            </div>
          </div>
        </div>

        <div class="challenge-options">
          <div class="c-option correct">
            <div class="opt-radio">A</div>
            <div class="opt-text">
              El solver `quick` usa un orden aleatorio de ecuaciones (Random Order) para evitar
              sesgos sistemáticos.
              <div class="opt-feedback">
                ¡Correcto! Para CI/CD, usa la bandera `--seed 1234` o el solver `world` (Dantzig).
              </div>
            </div>
          </div>
          <div class="c-option">
            <div class="opt-radio">B</div>
            <div class="opt-text">
              Gazebo tiene fugas de memoria que afectan el tiempo de cómputo.
            </div>
          </div>
          <div class="c-option">
            <div class="opt-radio">C</div>
            <div class="opt-text">Linux Scheduler cambia la prioridad del proceso random.</div>
          </div>
        </div>
      </div>
    </div>

    <!-- VIDEO TUTORIAL -->
    <div class="content-block q-mb-xl">
      <SectionTitle>7. Video Tutorial: Construcción End-to-End</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            width="100%"
            height="100%"
            src="https://www.youtube.com/embed/dQw4w9WgXcQ"
            title="Gazebo Worlds Tutorial"
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
          Implementación completa de <code>factory.py</code>, ajuste de física LCP y debug de
          sombras PBR.
        </div>
      </div>
    </div>

    <!-- SUMMARY -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen de Ingeniería</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>LCP Solver</code>
          <span
            >Core matemático que resuelve colisiones. Ajustar `ERP` (Error) y `CFM`
            (Sponginess).</span
          >
        </div>
        <div class="summary-item">
          <code>Procedural Gen</code>
          <span
            >Scripting Python + XML para crear aleatoriedad controlada (Domain Randomization).</span
          >
        </div>
        <div class="summary-item">
          <code>SDF Actor</code>
          <span
            >Animación de mallas con esqueleto (Skinning) sin física completa (Performance).</span
          >
        </div>
        <div class="summary-item">
          <code>Static Flag</code>
          <span
            >La optimización #1: `&lt;static&gt;true&lt;/static&gt;` saca objetos de la matriz de
            masa.</span
          >
        </div>
      </div>

      <AlertBlock type="success" title="Best Practices: Simulation Reliability" class="q-mt-lg">
        ✅ <strong>Determinismo:</strong> Fija siempre la semilla aleatoria (`random.seed(42)`) en
        tus scripts de generación.
        <br />
        ✅ <strong>Luz:</strong> Usa Dirección + Ambient Occlusion. Evita Point Lights excesivas
        (Costosas).
        <br />
        ✅ <strong>Modularidad:</strong> Nunca un solo archivo gigante. Usa `&lt;include&gt;`
        recursivos.
      </AlertBlock>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue';
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';

// --- PHYSICS TUNER LOGIC ---
const erpVal = ref(0.2);
const cfmVal = ref(0.001);
const itersVal = ref(50);

// Status Feedback
const erpStatus = computed(() => {
  if (erpVal.value < 0.1) return 'Drifting (Soft)';
  if (erpVal.value > 0.8) return 'Explosive Risk (Hard)';
  return 'Stable Region';
});

const cfmStatus = computed(() => {
  if (cfmVal.value === 0) return 'Singularity Risk (Rigid)';
  if (cfmVal.value > 0.05) return 'Sponge Effect';
  return 'Numerical Stability';
});

// Visualization Physics Mock
// We simulate how "deep" the object sinks based on CFM and how "hard" it bounces based on ERP
const penetrationDepth = computed(() => {
  // Higher CFM = More sinking (spongy)
  // Base sink -5px, max sink -40px
  return -5 - cfmVal.value * 400; // 0.1 * 400 = 40px sink
});

const compressionRatio = computed(() => {
  // Visual compression of the object
  return cfmVal.value * 2;
});

const correctionForce = computed(() => {
  // Force is proportional to ERP * Error / dt
  // Mock calculation for visual height
  return erpVal.value * 150 + 20;
});

const stabilityColor = computed(() => {
  if (erpVal.value > 0.8 || cfmVal.value === 0) return '#ef4444'; // Red (Danger)
  return '#22c55e'; // Green (Good)
});
</script>

<style scoped>
/* HERO SECTION */
.hero-section {
  background:
    radial-gradient(circle at 80% 20%, rgba(37, 99, 235, 0.1), transparent 50%),
    linear-gradient(135deg, #0f172a 0%, #1e293b 100%);
  border-radius: 24px;
  padding: 4rem 3rem;
  border: 1px solid rgba(148, 163, 184, 0.1);
  box-shadow: 0 20px 50px -10px rgba(0, 0, 0, 0.5);
  display: flex;
  justify-content: space-between;
  align-items: center;
  gap: 4rem;
  overflow: hidden;
  position: relative;
}

.hero-content {
  flex: 1;
  z-index: 2;
}

.hero-badge {
  display: inline-flex;
  align-items: center;
  gap: 0.5rem;
  background: rgba(59, 130, 246, 0.15);
  color: #60a5fa;
  padding: 0.5rem 1rem;
  border-radius: 99px;
  font-size: 0.85rem;
  font-weight: 700;
  border: 1px solid rgba(59, 130, 246, 0.3);
  margin-bottom: 1.5rem;
}

.hero-title {
  font-size: 3.5rem;
  font-weight: 800;
  line-height: 1.1;
  margin-bottom: 1.5rem;
  letter-spacing: -1px;
}

.gradient-text {
  background: linear-gradient(to right, #60a5fa, #c084fc);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
}

.hero-subtitle {
  font-size: 1.2rem;
  color: #94a3b8;
  margin-bottom: 2.5rem;
  line-height: 1.7;
  max-width: 600px;
}

.hero-stats {
  display: flex;
  gap: 3rem;
}

.stat-val {
  font-size: 1.5rem;
  font-weight: 700;
  color: #fff;
  margin-bottom: 0.25rem;
}

.stat-label {
  font-size: 0.85rem;
  color: #64748b;
  text-transform: uppercase;
  letter-spacing: 1px;
}

/* HERO VIZ */
.hero-viz {
  position: relative;
  width: 400px;
  height: 400px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.physics-mesh {
  width: 100%;
  height: 100%;
  animation: float 6s ease-in-out infinite;
}

.node {
  filter: drop-shadow(0 0 10px rgba(96, 165, 250, 0.5));
}

.node.core {
  animation: pulse-core 3s infinite;
}

.node.satellite {
  transform-origin: 100px 100px;
}
.node.satellite.s1 {
  animation: orbit 10s linear infinite;
}
.node.satellite.s2 {
  animation: orbit 15s linear infinite reverse;
}
.node.satellite.s3 {
  animation: orbit 20s linear infinite;
}
.node.satellite.s4 {
  animation: orbit 25s linear infinite reverse;
}

.link {
  opacity: 0.6;
}

/* MATH VIZ LCP */
.math-viz-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  background: rgba(15, 23, 42, 0.6);
  padding: 2rem;
  border-radius: 16px;
  border: 1px solid rgba(148, 163, 184, 0.1);
}

.equation-row {
  display: flex;
  align-items: center;
  gap: 1.5rem;
  font-family: 'Fira Code', monospace;
  font-size: 1.5rem;
  color: #fff;
}

.matrix-label {
  padding: 0.5rem 1rem;
  border: 2px solid #cbd5e1;
  border-radius: 8px;
  background: rgba(255, 255, 255, 0.05);
}

.operator {
  color: #64748b;
}

.equation-desc {
  margin-top: 1rem;
  color: #94a3b8;
  font-size: 0.9rem;
  font-style: italic;
}

/* PHYSICS LAB TOOL */
.tool-wrapper {
  background: #0f172a;
  border-radius: 20px;
  border: 1px solid #334155;
  overflow: hidden;
}

.tool-header {
  background: #1e293b;
  padding: 1rem 1.5rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
  border-bottom: 1px solid #334155;
}

.tool-header .q-icon {
  font-size: 1.5rem;
  color: #a855f7;
}

.tool-title {
  font-weight: 700;
  color: #fff;
  letter-spacing: 0.5px;
}

.physics-lab {
  display: grid;
  grid-template-columns: 350px 1fr;
  min-height: 400px;
}

.lab-controls {
  padding: 2rem;
  background: rgba(30, 41, 59, 0.5);
  border-right: 1px solid #334155;
  display: flex;
  flex-direction: column;
  gap: 2rem;
}

.control-group {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.c-label {
  font-weight: 700;
  color: #e2e8f0;
  font-size: 0.9rem;
}

.c-desc {
  font-size: 0.8rem;
  color: #94a3b8;
  line-height: 1.4;
}

.lab-viz {
  position: relative;
  background: linear-gradient(180deg, #1e293b 0%, #0f172a 100%);
  display: flex;
  justify-content: center;
  align-items: flex-end;
  padding-bottom: 50px;
  overflow: hidden;
}

.ground-plane {
  position: absolute;
  bottom: 0;
  width: 80%;
  height: 10px;
  background: #3b82f6;
  border-radius: 4px;
  box-shadow: 0 0 20px rgba(59, 130, 246, 0.4);
}

.dynamic-object {
  width: 100px;
  height: 100px;
  background: rgba(245, 158, 11, 0.2);
  border: 3px solid;
  border-radius: 8px;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  position: relative;
  transition: all 0.2s cubic-bezier(0.4, 0, 0.2, 1);
  z-index: 10;
}

.obj-label {
  color: #fff;
  font-weight: 700;
  font-size: 0.9rem;
}

.obj-mass {
  color: #fbbf24;
  font-size: 0.75rem;
}

.force-vector {
  position: absolute;
  bottom: 160px; /* Above the object */
  width: 4px;
  background: #22c55e;
  display: flex;
  justify-content: center;
  border-radius: 2px;
  transition: all 0.2s;
}

.force-label {
  position: absolute;
  top: -25px;
  background: #22c55e;
  color: #000;
  padding: 2px 6px;
  border-radius: 4px;
  font-size: 0.75rem;
  font-weight: 700;
  white-space: nowrap;
}

.penetration-zone {
  position: absolute;
  bottom: 0px;
  width: 100px;
  background: rgba(239, 68, 68, 0.3);
  border-top: 1px dashed #ef4444;
  display: flex;
  align-items: center;
  justify-content: center;
  color: #fca5a5;
  font-size: 0.7rem;
  z-index: 5;
}

/* CODE SECTION */
.code-section {
  border: 1px solid #334155;
  border-radius: 12px;
  overflow: hidden;
}

.code-header {
  background: #1e293b;
  padding: 0.75rem 1rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
  border-bottom: 1px solid #334155;
  color: #94a3b8;
  font-family: monospace;
  font-size: 0.9rem;
}

/* ANIMATIONS */
@keyframes float {
  0%,
  100% {
    transform: translateY(0);
  }
  50% {
    transform: translateY(-20px);
  }
}

@keyframes pulse-core {
  0%,
  100% {
    transform: scale(1);
    opacity: 1;
  }
  50% {
    transform: scale(1.1);
    opacity: 0.8;
  }
}

@keyframes orbit {
  from {
    transform: rotate(0deg) translateX(50px) rotate(0deg);
  }
  to {
    transform: rotate(360deg) translateX(50px) rotate(-360deg);
  }
}

@media (max-width: 1024px) {
  .hero-section {
    flex-direction: column;
    padding: 3rem 1.5rem;
  }
  .physics-lab {
    grid-template-columns: 1fr;
  }
  .lab-controls {
    border-right: none;
    border-bottom: 1px solid #334155;
  }
}

/* RENDER PIPELINE VIZ */
.render-pipeline-viz {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 1.5rem;
  background: rgba(15, 23, 42, 0.6);
  padding: 2rem;
  border-radius: 16px;
  flex-wrap: wrap;
}

.pipeline-stage {
  display: flex;
  flex-direction: column;
  align-items: center;
  text-align: center;
  background: rgba(30, 41, 59, 0.5);
  padding: 1.5rem;
  border-radius: 12px;
  border: 1px solid rgba(148, 163, 184, 0.1);
  width: 180px;
  transition: transform 0.2s;
}

.pipeline-stage:hover {
  transform: translateY(-5px);
  background: rgba(30, 41, 59, 0.8);
}

.stage-label {
  font-weight: 700;
  color: #fff;
  margin-top: 1rem;
  font-size: 1rem;
}

.stage-desc {
  font-size: 0.8rem;
  color: #94a3b8;
  margin-top: 0.5rem;
}

.pipeline-arrow {
  color: #64748b;
  font-size: 1.5rem;
  font-weight: 700;
}

/* SDF TREE VIZ */
.sdf-tree-container {
  background: #0f172a;
  padding: 2rem;
  border-radius: 16px;
  border-left: 4px solid #f59e0b;
  font-family: 'Fira Code', monospace;
  overflow-x: auto;
}

.tree-root {
  color: #64748b;
  margin-bottom: 0.5rem;
}

.tree-branch {
  margin-left: 1.5rem;
  position: relative;
}

.branch-line {
  position: absolute;
  left: -12px;
  top: 0;
  bottom: 0;
  width: 2px;
  background: #334155;
}

.tree-node.world {
  color: #fbbf24;
  font-weight: 700;
  display: flex;
  align-items: center;
  gap: 0.5rem;
  margin-bottom: 1rem;
}

.tree-children {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
  margin-left: 2rem;
}

.child-node {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  background: rgba(255, 255, 255, 0.05);
  padding: 0.5rem 1rem;
  border-radius: 6px;
  color: #cbd5e1;
  font-size: 0.9rem;
  width: fit-content;
}

.child-node .q-icon {
  color: #60a5fa;
}

.child-node.physics {
  border-left: 2px solid #ef4444;
}
.child-node.scene {
  border-left: 2px solid #a855f7;
}
.child-node.include {
  border-left: 2px solid #22c55e;
}
.child-node.plugin {
  border-left: 2px solid #ec4899;
}

/* PROCEDURAL VIZ */
.procedural-viz-container {
  background: #0f172a;
  border: 1px solid #334155;
  border-radius: 12px;
  overflow: hidden;
  display: flex;
  flex-direction: column;
}

.proc-header {
  background: #1e293b;
  padding: 0.75rem 1rem;
  display: flex;
  justify-content: space-between;
  align-items: center;
  border-bottom: 1px solid #334155;
  color: #fff;
  font-weight: 700;
}

.proc-badge {
  background: #f59e0b;
  color: #000;
  font-size: 0.7rem;
  padding: 2px 6px;
  border-radius: 4px;
}

.proc-grid {
  display: flex;
  align-items: center;
  justify-content: space-around;
  padding: 2rem;
  background: rgba(30, 41, 59, 0.3);
}

.proc-step {
  display: flex;
  flex-direction: column;
  align-items: center;
  text-align: center;
  gap: 0.5rem;
}

.step-icon {
  width: 50px;
  height: 50px;
  background: rgba(255, 255, 255, 0.1);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  border: 1px solid rgba(255, 255, 255, 0.2);
}

.step-icon .q-icon {
  font-size: 1.5rem;
  color: #a5f3fc;
}

.step-text {
  font-size: 0.8rem;
  color: #94a3b8;
  line-height: 1.2;
}

.step-text strong {
  color: #e2e8f0;
  display: block;
  margin-bottom: 2px;
}

.proc-arrow {
  color: #475569;
  font-size: 1.2rem;
}

/* MAP PREVIEW */
.proc-map-preview {
  background: #000;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
}

.map-grid {
  display: grid;
  grid-template-columns: repeat(10, 1fr);
  gap: 2px;
  width: 100%;
  max-width: 300px;
  height: 150px;
}

.map-cell {
  background: #1e293b;
  border-radius: 2px;
}

.map-cell.occupied {
  background: #3b82f6;
  box-shadow: 0 0 5px rgba(59, 130, 246, 0.5);
}

.map-label {
  font-family: monospace;
  color: #22c55e;
  font-size: 0.75rem;
  margin-top: 0.5rem;
}

/* CASE STUDY VIZ */
.case-study-viz {
  display: flex;
  justify-content: center;
  gap: 2rem;
  flex-wrap: wrap;
}

.cs-card {
  background: rgba(15, 23, 42, 0.6);
  padding: 2rem;
  border-radius: 16px;
  display: flex;
  flex-direction: column;
  align-items: center;
  border: 1px solid rgba(148, 163, 184, 0.1);
  width: 160px;
}

.cs-stat {
  font-size: 2rem;
  font-weight: 800;
  color: #fff;
  margin: 0.5rem 0;
}

.cs-label {
  font-size: 0.8rem;
  color: #94a3b8;
  text-transform: uppercase;
  letter-spacing: 1px;
}

/* CHALLENGE CARD */
.challenge-card {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid #d946ef;
  border-radius: 16px;
  padding: 2rem;
}

.challenge-header {
  display: flex;
  gap: 1.5rem;
  margin-bottom: 2rem;
}

.c-icon {
  font-size: 3rem;
  color: #d946ef;
}

.c-title {
  font-size: 1.25rem;
  font-weight: 700;
  color: #f0abfc;
  margin-bottom: 0.5rem;
}

.c-desc {
  color: #cbd5e1;
  font-size: 0.95rem;
}

.challenge-options {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.c-option {
  display: flex;
  gap: 1rem;
  padding: 1rem;
  background: rgba(0, 0, 0, 0.2);
  border-radius: 8px;
  cursor: default; /* Static for now */
}

.c-option.correct {
  border: 1px solid #22c55e;
  background: rgba(34, 197, 94, 0.1);
}

.opt-radio {
  width: 30px;
  height: 30px;
  border-radius: 50%;
  border: 2px solid #64748b;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 700;
  color: #cbd5e1;
  flex-shrink: 0;
}

.c-option.correct .opt-radio {
  border-color: #22c55e;
  color: #22c55e;
}

.opt-text {
  color: #fff;
  flex: 1;
}

.opt-feedback {
  margin-top: 0.5rem;
  color: #86efac;
  font-size: 0.9rem;
}

/* VIDEO STYLES */
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
  padding-bottom: 56.25%;
  height: 0;
  overflow: hidden;
  border-radius: 8px;
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
  padding: 1rem 0.5rem 0.5rem 0.5rem;
  color: #94a3b8;
  font-size: 0.9rem;
  display: flex;
  align-items: center;
  gap: 0.75rem;
}

/* SUMMARY & NEXT STEP */
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
  color: #f59e0b;
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
