<template>
  <div class="rmw-visualizer">
    <div class="viz-header">
      <div class="viz-title">
        <q-icon name="hub" size="sm" class="q-mr-sm" />
        Data Path Visualizer
      </div>
      <div class="viz-controls">
        <q-btn-group unelevated>
          <q-btn
            :color="mode === 'udp' ? 'blue' : 'grey-9'"
            label="Network (UDP)"
            size="sm"
            @click="mode = 'udp'"
            class="mode-btn"
          />
          <q-btn
            :color="mode === 'shm' ? 'green' : 'grey-9'"
            label="Shared Memory"
            size="sm"
            @click="mode = 'shm'"
            class="mode-btn"
          />
        </q-btn-group>
      </div>
    </div>

    <div class="viz-container">
      <!-- STATIC LAYERS -->
      <div class="layer app-layer">
        <div class="layer-label">User Application (ROS 2 Node)</div>
        <div class="data-packet" :class="{ animating: isAnimating, shm: mode === 'shm' }">DATA</div>
      </div>

      <div class="layer rcl-layer">
        <div class="layer-label">RCL (Format Agnostic)</div>
      </div>

      <div class="layer rmw-layer">
        <div class="layer-label">
          RMW Implementation ({{ mode === 'udp' ? 'FastDDS / Cyclone' : 'FastDDS SHM' }})
        </div>
      </div>

      <!-- DYNAMIC ROUTING -->
      <div class="routing-container">
        <!-- UDP ROUTE -->
        <div v-if="mode === 'udp'" class="route-udp">
          <div class="layer dds-layer">
            <div class="layer-label">DDS Protocol</div>
          </div>
          <div class="layer os-layer negative">
            <div class="layer-label">
              <q-icon name="warning" color="orange" size="xs" />
              OS Kernel (Socket Buffer Copy)
            </div>
          </div>
          <div class="layer network-layer negative">
            <div class="layer-label">Network Interface (NIC)</div>
          </div>
        </div>

        <!-- SHM ROUTE -->
        <div v-if="mode === 'shm'" class="route-shm">
          <div class="layer shm-layer positive">
            <div class="layer-label">
              <q-icon name="bolt" color="white" size="xs" />
              Shared Memory Segment (Zero Copy)
            </div>
            <div class="shm-visual">
              <div class="pointer p1">Ptr</div>
              <div class="memory-block">DATA</div>
              <div class="pointer p2">Ptr</div>
            </div>
          </div>
        </div>
      </div>

      <!-- DESTINATION -->
      <div class="layer app-layer dest">
        <div class="layer-label">Subscriber Node</div>
      </div>
    </div>

    <div class="viz-stats">
      <div class="stat-item">
        <div class="stat-label">Latencia Est.</div>
        <div class="stat-value" :class="mode === 'shm' ? 'text-green-4' : 'text-blue-4'">
          {{ mode === 'shm' ? '~5 μs' : '~500 μs' }}
        </div>
      </div>
      <div class="stat-item">
        <div class="stat-label">CPU Usage</div>
        <div class="stat-value" :class="mode === 'shm' ? 'text-green-4' : 'text-orange-4'">
          {{ mode === 'shm' ? 'Very Low' : 'Medium (Serialization)' }}
        </div>
      </div>
      <div class="stat-item">
        <div class="stat-label">Mechanism</div>
        <div class="stat-value text-white">
          {{ mode === 'shm' ? 'Pointer Passing' : 'Socket I/O + Serialization' }}
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, watch, onMounted } from 'vue';

const mode = ref<'udp' | 'shm'>('udp');
const isAnimating = ref(false);

const startAnimation = () => {
  isAnimating.value = false;
  setTimeout(() => {
    isAnimating.value = true;
  }, 50);
};

watch(mode, () => {
  startAnimation();
});

onMounted(() => {
  startAnimation();
});
</script>

<style scoped>
.rmw-visualizer {
  background: rgba(15, 23, 42, 0.9);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  font-family: 'Inter', sans-serif;
  max-width: 600px;
  margin: 0 auto;
}

.viz-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 1rem;
  background: rgba(0, 0, 0, 0.4);
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}

.viz-title {
  color: #f1f5f9;
  font-weight: 600;
  display: flex;
  align-items: center;
}

.viz-container {
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
  position: relative;
  min-height: 400px;
}

.layer {
  border: 1px solid rgba(255, 255, 255, 0.1);
  background: rgba(30, 41, 59, 0.8);
  border-radius: 6px;
  padding: 0.75rem;
  text-align: center;
  position: relative;
  transition: all 0.3s ease;
}

.layer-label {
  font-size: 0.85rem;
  color: #94a3b8;
  font-weight: 500;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 0.5rem;
}

.app-layer {
  background: rgba(37, 99, 235, 0.2);
  border-color: rgba(37, 99, 235, 0.4);
}

.app-layer.dest {
  margin-top: auto;
}

.layer.negative {
  background: rgba(239, 68, 68, 0.1);
  border-color: rgba(239, 68, 68, 0.3);
}

.layer.positive {
  background: rgba(34, 197, 94, 0.1);
  border-color: rgba(34, 197, 94, 0.3);
  padding: 1.5rem;
}

.rmw-layer {
  background: rgba(100, 116, 139, 0.2);
}

.data-packet {
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  background: #f1f5f9;
  color: #0f172a;
  font-weight: 800;
  font-size: 0.7rem;
  padding: 0.25rem 0.5rem;
  border-radius: 4px;
  z-index: 10;
  opacity: 0;
  width: 50px;
}

.data-packet.animating {
  animation: data-flow-udp 2s cubic-bezier(0.4, 0, 0.2, 1) infinite;
}

.data-packet.animating.shm {
  animation: data-flow-shm 1.5s cubic-bezier(0.4, 0, 0.2, 1) infinite;
}

@keyframes data-flow-udp {
  0% {
    top: 30px;
    opacity: 1;
    transform: translate(-50%, 0) scale(1);
  }
  20% {
    transform: translate(-50%, 0) scale(0.9);
  }
  40% {
    transform: translate(-50%, 0) scale(0.9);
  } /* Kernel squeeze */
  60% {
    transform: translate(-50%, 0) scale(0.9);
  } /* Network squeeze */
  80% {
    transform: translate(-50%, 0) scale(1);
  }
  95% {
    top: 350px;
    opacity: 1;
  }
  100% {
    top: 350px;
    opacity: 0;
  }
}

@keyframes data-flow-shm {
  0% {
    top: 30px;
    opacity: 1;
    background: #f1f5f9;
  }
  30% {
    top: 120px;
    background: #22c55e;
    color: white;
    content: 'PTR';
  } /* Transforms to pointer */
  70% {
    top: 200px;
  }
  90% {
    top: 350px;
    opacity: 1;
    background: #f1f5f9;
    color: #0f172a;
  }
  100% {
    top: 350px;
    opacity: 0;
  }
}

.shm-visual {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 1rem;
  margin-top: 0.5rem;
}

.memory-block {
  background: #22c55e;
  color: white;
  padding: 0.5rem 1rem;
  border-radius: 4px;
  font-family: monospace;
  font-weight: bold;
  box-shadow: 0 0 15px rgba(34, 197, 94, 0.4);
}

.pointer {
  font-size: 0.7rem;
  color: #86efac;
  font-family: monospace;
  position: relative;
}

.pointer::before {
  content: '→';
  display: block;
  font-size: 1rem;
  text-align: center;
}

.viz-stats {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  padding: 1rem;
  background: rgba(0, 0, 0, 0.2);
  gap: 1rem;
}

.stat-item {
  text-align: center;
}

.stat-label {
  font-size: 0.7rem;
  color: #64748b;
  text-transform: uppercase;
  letter-spacing: 0.05em;
  margin-bottom: 0.25rem;
}

.stat-value {
  font-weight: 700;
  font-size: 0.9rem;
}
</style>
