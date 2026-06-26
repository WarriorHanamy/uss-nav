import { Canvas } from "@react-three/fiber";
import { OrbitControls, PerspectiveCamera } from "@react-three/drei";
import { TrajectoryLine3D } from "./TrajectoryLine3D";
import type { TestRunDisplay } from "../lib/types";
import { useMemo } from "react";

function Grid() {
  return (
    <gridHelper args={[40, 40, "#333", "#222"]} position={[0, -0.5, 0]} />
  );
}

function Axes() {
  return <axesHelper args={[2]} />;
}

interface SceneProps {
  tests: TestRunDisplay[];
}

export function Scene3D({ tests }: SceneProps) {
  const trajectories = useMemo(
    () => tests.map((t) => <TrajectoryLine3D key={t.id} test={t} />),
    [tests],
  );

  return (
    <Canvas style={{ width: "100%", height: "100%" }}>
      <PerspectiveCamera makeDefault position={[15, 10, 15]} />
      <ambientLight intensity={0.5} />
      <directionalLight position={[10, 10, 5]} intensity={0.8} />
      <Grid />
      <Axes />
      {trajectories}
      <OrbitControls
        enableDamping
        dampingFactor={0.1}
        maxDistance={100}
        minDistance={1}
      />
    </Canvas>
  );
}
