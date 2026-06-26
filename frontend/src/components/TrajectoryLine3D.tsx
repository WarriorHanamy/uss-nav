import { useMemo } from "react";
import { Line } from "@react-three/drei";
import type { TestRunDisplay } from "../lib/types";

interface Props {
  test: TestRunDisplay;
}

export function TrajectoryLine3D({ test }: Props) {
  const points = test.positions;

  const pts = useMemo(() => points.map((p) => [p[0], p[1], p[2]] as [number, number, number]), [points]);

  if (pts.length < 2) return null;

  return (
    <Line
      points={pts}
      color={test.color}
      lineWidth={2}
      transparent
      opacity={0.8}
    />
  );
}
