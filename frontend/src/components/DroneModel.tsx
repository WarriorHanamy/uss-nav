import { useRef, useMemo } from "react";
import { useFrame } from "@react-three/fiber";
import * as THREE from "three";

interface Props {
  position: [number, number, number];
  orientation: [number, number, number, number];
  speed: number;
}

/**
 * A quadrotor drone built from Three.js primitives.
 * Orientation uses IEEE ROS convention:
 *   quaternion (w, x, y, z) = THREE (x, y, z, w)
 */
export function DroneModel({ position, orientation, speed }: Props) {
  const groupRef = useRef<THREE.Group>(null);
  const prop1 = useRef<THREE.Mesh>(null);
  const prop2 = useRef<THREE.Mesh>(null);
  const prop3 = useRef<THREE.Mesh>(null);
  const prop4 = useRef<THREE.Mesh>(null);

  // Convert ROS quaternion (w,x,y,z) to THREE (x,y,z,w)
  const quat = useMemo(
    () => new THREE.Quaternion(orientation[1], orientation[2], orientation[3], orientation[0]),
    [orientation],
  );

  useFrame(() => {
    if (!groupRef.current) return;
    groupRef.current.position.set(position[0], position[1], position[2]);
    groupRef.current.quaternion.copy(quat);

    // Spin propellers proportional to speed
    const rpm = speed * 20;
    [prop1, prop2, prop3, prop4].forEach((p) => {
      if (p.current) p.current.rotation.z += rpm * 0.016;
    });
  });

  const armLen = 0.25;
  const bodyR = 0.06;
  const propR = 0.12;

  return (
    <group ref={groupRef}>
      {/* Body */}
      <mesh position={[0, 0, 0]}>
        <cylinderGeometry args={[bodyR, bodyR, 0.08, 12]} />
        <meshStandardMaterial color="#444" />
      </mesh>

      {/* Arms (X-configuration) */}
      {[
        [ armLen, 0, 0], [-armLen, 0, 0],
        [0,  armLen, 0], [0, -armLen, 0],
      ].map((pos, i) => (
        <mesh key={`arm-${i}`} position={[pos[0] * 0.5, pos[1] * 0.5, 0]}>
          <cylinderGeometry args={[0.015, 0.015, armLen, 6]} />
          <meshStandardMaterial color="#666" />
        </mesh>
      ))}

      {/* Propellers */}
      {[
        [ armLen, 0, prop1], [-armLen, 0, prop2],
        [0,  armLen, prop3], [0, -armLen, prop4],
      ].map(([x, y, ref], i) => (
        <mesh key={`prop-${i}`} ref={ref as React.Ref<THREE.Mesh>} position={[x as number, y as number, 0.02]}>
          <circleGeometry args={[propR, 16]} />
          <meshStandardMaterial color="#88f" transparent opacity={0.6} side={THREE.DoubleSide} />
        </mesh>
      ))}
    </group>
  );
}
