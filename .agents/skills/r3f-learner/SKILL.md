---
name: r3f-learner
description: Core React Three Fiber (R3F) patterns used in this project — component mapping, Canvas setup, drei helpers, data-to-3D flow, and performance rules. Use when writing or reviewing 3D scene components, trajectory rendering, or WebSocket-fed visualizations.
---

# React Three Fiber Learner

## 1. Three.js → R3F Component Mapping

| Three.js API                | R3F Component         | Notes                                  |
| --------------------------- | --------------------- | -------------------------------------- |
| `THREE.Mesh`                | `<mesh>`              | Default geometry + material container  |
| `THREE.Mesh` + BoxGeometry  | `<mesh><boxGeometry args={[1,1,1]} /><meshBasicMaterial color="red" /></mesh>` | Geometries and materials are children |
| `THREE.BufferGeometry`      | `<bufferGeometry>`    | For custom vertex data                 |
| `THREE.Points`              | `<points>`            | With `<bufferGeometry>` + `<pointsMaterial>` |
| `THREE.Line`                | `<Line>` (from drei)  | Prefer drei `<Line>` over raw `<line>` |
| `THREE.PerspectiveCamera`   | `<PerspectiveCamera>` | Use `makeDefault` to override Canvas's default |
| `THREE.Scene`               | `<Canvas>`            | Root container, auto-creates scene     |
| `THREE.AmbientLight`        | `<ambientLight>`      | lowercase camelCase                    |
| `THREE.DirectionalLight`    | `<directionalLight>`  | lowercase camelCase                    |
| `THREE.AxesHelper`          | `<axesHelper>`        | `args={[size]}`                        |
| `THREE.GridHelper`          | `<gridHelper>`        | `args={[size, divisions, colorCenter, colorGrid]}`  |
| `THREE.OrbitControls`       | `<OrbitControls>` (drei) | With `enableDamping` for smooth pan  |
| `THREE.Scene.add(object)`   | JSX child placement    | Parent-child in JSX tree = scene graph |

## 2. Canvas Setup (Project Standard)

```tsx
import { Canvas } from "@react-three/fiber";
import { OrbitControls, PerspectiveCamera } from "@react-three/drei";

function Scene() {
  return (
    <Canvas style={{ width: "100%", height: "100%" }}>
      <PerspectiveCamera makeDefault position={[15, 10, 15]} />
      <ambientLight intensity={0.5} />
      <directionalLight position={[10, 10, 5]} intensity={0.8} />
      <OrbitControls
        enableDamping
        dampingFactor={0.1}
        maxDistance={100}
        minDistance={1}
      />
    </Canvas>
  );
}
```

Rules:
- Canvas at 100% viewport (no fixed pixel size)
- `makeDefault` on `PerspectiveCamera` replaces Canvas's default camera
- OrbitControls always with `enableDamping` in this project
- No `<Suspense>` wrapping (no loaded assets yet)
- No `gl` props on Canvas (use defaults)
- Helpers (`gridHelper`, `axesHelper`) in dedicated local components

## 3. Declarative Props

R3F accepts Three.js properties as JSX attributes with smart auto-conversion:

```tsx
// Arrays → THREE.Vector3 / THREE.Color
<mesh position={[1, 0, 0]} rotation={[0, Math.PI / 2, 0]} scale={1.5} />

// Colors → THREE.Color (string, hex, array)
<meshStandardMaterial color="#ff6b6b" />

// Constructor args use `args` prop
<sphereGeometry args={[radius, widthSeg, heightSeg]} />

// Named props = setter calls
<mesh visible={false} castShadow />
```

Rules:
- `position`, `scale`: use array literal `[x, y, z]`
- `color`: use CSS hex string `"#ff6b6b"` (matching project COLORS array)
- `rotation`: radians, array `[x, y, z]`
- Constructor arguments always in `args={[...]}` prop

## 4. Drei Helpers (Currently Used)

| Import                  | Component              | Project usage                    |
| ----------------------- | ---------------------- | -------------------------------- |
| `@react-three/drei`     | `<OrbitControls>`      | Camera control with damping      |
| `@react-three/drei`     | `<PerspectiveCamera>`  | Declarative camera setup         |
| `@react-three/drei`     | `<Line>`               | Trajectory rendering             |

### `Line` Usage (TrajectoryLine3D pattern)

```tsx
import { Line } from "@react-three/drei";
import { useMemo } from "react";

function TrajectoryLine({ points, color }: Props) {
  const pts = useMemo(
    () => points.map((p) => [p[0], p[1], p[2]] as [number, number, number]),
    [points],
  );

  if (pts.length < 2) return null;

  return (
    <Line
      points={pts}
      color={color}
      lineWidth={2}
      transparent
      opacity={0.8}
    />
  );
}
```

`Line` from drei is preferred over raw `<line>` because:
- Auto-creates `THREE.BufferGeometry` from array points
- Supports color, opacity, lineWidth uniformly
- No manual geometry/attribute setup needed

### Additional Drei Components for Future Use

| Component          | Purpose                              | Import                     |
| ------------------ | ------------------------------------ | -------------------------- |
| `<Html>`           | Overlay DOM elements in 3D space     | `@react-three/drei`        |
| `<Text>`           | 3D text (troika-three-text)           | `@react-three/drei`        |
| `<GizmoHelper>`    | Viewport orientation gizmo           | `@react-three/drei`        |
| `<Stats>`          | FPS/performance overlay              | `@react-three/drei`        |
| `<Grid>`           | Infinite grid (v9+)                  | `@react-three/drei`        |
| `<Float>`          | Floating animation helper            | `@react-three/drei`        |
| `<TransformControls>` | Interactive transform manipulator | `@react-three/drei`        |

## 5. Data → 3D Flow

This project uses **props-driven** rendering (not `useFrame`):

```
WebSocket → App.tsx (state: Map<string, TestRunDisplay>)
    → Array.from(tests.values())
    → Scene3D (useMemo: tests.map → <TrajectoryLine3D>)
    → TrajectoryLine3D (useMemo: positions.map → points[])
    → <Line points={pts} />
```

Rules:
- **State lives in App.tsx**, passed down as props to Scene3D
- **`useMemo` guards re-renders** at every level:
  - Scene3D: re-creates trajectory elements only when `tests` array reference changes
  - TrajectoryLine3D: re-computes point arrays only when `positions` reference changes
- **No `useFrame`** needed — all updates come from React state changes
- **No `useThree`** in current patterns (but can use in future for imperative access)
- Positions capped at 5000 samples (`slice(-5000)`) to prevent memory growth

### When to use props-driven vs useFrame

| Pattern            | When to use                                  |
| ------------------ | -------------------------------------------- |
| Props-driven       | Data comes from external source (WebSocket, fetch, React state) |
| `useFrame`         | Continuous animation (rotation oscillation, particle systems)   |
| `useThree`         | Need access to camera, renderer, or scene imperatively          |

## 6. Performance Rules

1. **Cap array sizes** — positions ≤ 5000, plan results ≤ 1000 (see App.tsx)
2. **useMemo at every level** — mapping data to R3F elements should be memoized
3. **Prefer drei `<Line>` over raw `<line>`** — fewer draw calls for multi-segment lines
4. **Avoid inline functions in render** — use `useMemo`/`useCallback` for children
5. **No re-create geometries per frame** — if using `useFrame`, mutate attributes in-place
6. **Color is a string prop on MeshStandardMaterial** — React reconciler handles disposal
7. **`transparent + opacity` is acceptable** — this project uses it sparsely

## 7. Common Pitfalls

| Mistake                                         | Fix                                                       |
| ----------------------------------------------- | --------------------------------------------------------- |
| R3F component outside `<Canvas>`                | All `<mesh>`, `<Line>`, `<ambientLight>` must be inside `<Canvas>` |
| `NaN` or `Infinity` in position array            | Validate/filter before passing — Three.js won't error, just blank |
| Missing `key` prop in mapped R3F elements       | Add `key={test.id}` — R3F uses React reconciler to reuse objects |
| `useLoader` without `<Suspense>`                 | Wrap Canvas content in `<Suspense fallback={null}>`       |
| Mutating R3F props without new reference         | R3F uses shallow comparison — spread or new array/object  |
| Import from `three` instead of `@react-three/*`  | Use `@react-three/fiber` for components, `@react-three/drei` for helpers |
| Inline `position={new THREE.Vector3(...)}`      | Use array: `position={[x, y, z]}` (R3F auto-converts)    |

## 8. Project Conventions

- **Coordinate system**: Y-up (Three.js default)
- **Grid**: 40×40, center color `#333`, grid color `#222`, at y=-0.5
- **Axes**: 2-unit length
- **Camera default**: position [15, 10, 15], lookAt origin
- **Trajectory colors**: 8-color palette defined in App.tsx, assigned by index
- **Color format**: CSS hex strings (`"#ff6b6b"`)
- **Trajectories with < 2 points**: return `null` (skip rendering)
- **Package**: all imports from `@react-three/fiber` and `@react-three/drei` (not raw `three`)
- **No TypeScript path aliases** — relative imports everywhere
- **Class components**: never — function components only with hooks
