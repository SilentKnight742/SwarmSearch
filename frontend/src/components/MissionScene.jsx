import {
  Html,
  Line,
  OrbitControls,
} from "@react-three/drei";

import {
  Canvas,
} from "@react-three/fiber";


const UAV_COLORS = {
  "UAV-1": "#55a9d8",
  "UAV-2": "#d7b75b",
  "UAV-3": "#8f7fcb",
};


function SearchArea({
  width,
  height,
}) {
  return (
    <>
      <mesh
        position={[
          width / 2,
          -0.04,
          height / 2,
        ]}
        rotation={[
          -Math.PI / 2,
          0,
          0,
        ]}
      >
        <planeGeometry
          args={[
            width,
            height,
          ]}
        />

        <meshStandardMaterial
          color="#0b141c"
          transparent
          opacity={0.82}
        />
      </mesh>

      <Line
        points={[
          [0, 0.02, 0],
          [width, 0.02, 0],
          [
            width,
            0.02,
            height,
          ],
          [
            0,
            0.02,
            height,
          ],
          [0, 0.02, 0],
        ]}
        color="#42637b"
        lineWidth={1}
      />
    </>
  );
}


function PlannedRoute({
  name,
  segment,
}) {
  const color =
    UAV_COLORS[name] ??
    "#718392";

  const points =
    segment.waypoints.map(
      (waypoint) => [
        waypoint.east,
        0.07,
        waypoint.north,
      ],
    );

  if (points.length < 2) {
    return null;
  }

  const start =
    segment.start;

  const entry =
    segment.entry ??
    segment.waypoints[0];

  return (
    <>
      {start && entry && (
        <Line
          points={[
            [
              start.east,
              0.05,
              start.north,
            ],
            [
              entry.east,
              0.05,
              entry.north,
            ],
          ]}
          color="#5e6d77"
          lineWidth={1}
          transparent
          opacity={0.5}
          dashed
          dashScale={4}
          dashSize={0.5}
          gapSize={0.5}
        />
      )}

      <Line
        points={points}
        color={color}
        lineWidth={1.4}
        transparent
        opacity={0.78}
      />

      <mesh
        position={[
          entry.east,
          0.08,
          entry.north,
        ]}
        rotation={[
          Math.PI / 2,
          0,
          0,
        ]}
      >
        <ringGeometry
          args={[
            0.45,
            0.6,
            24,
          ]}
        />

        <meshBasicMaterial
          color={color}
        />
      </mesh>
    </>
  );
}


function Rotor({
  position,
  color,
}) {
  return (
    <group
      position={position}
    >
      <mesh>
        <cylinderGeometry
          args={[
            0.10,
            0.10,
            0.10,
            12,
          ]}
        />

        <meshStandardMaterial
          color={color}
        />
      </mesh>

      <mesh
        rotation={[
          Math.PI / 2,
          0,
          0,
        ]}
      >
        <torusGeometry
          args={[
            0.42,
            0.035,
            8,
            28,
          ]}
        />

        <meshStandardMaterial
          color={color}
          transparent
          opacity={0.8}
        />
      </mesh>
    </group>
  );
}


function Quadrotor({
  color,
}) {
  return (
    <group>
      <mesh>
        <boxGeometry
          args={[
            1.05,
            0.32,
            0.72,
          ]}
        />

        <meshStandardMaterial
          color={color}
          roughness={0.48}
          metalness={0.18}
        />
      </mesh>

      <mesh>
        <boxGeometry
          args={[
            3.0,
            0.07,
            0.10,
          ]}
        />

        <meshStandardMaterial
          color="#778591"
        />
      </mesh>

      <mesh>
        <boxGeometry
          args={[
            0.10,
            0.07,
            3.0,
          ]}
        />

        <meshStandardMaterial
          color="#778591"
        />
      </mesh>

      <Rotor
        position={[
          1.45,
          0.08,
          0,
        ]}
        color={color}
      />

      <Rotor
        position={[
          -1.45,
          0.08,
          0,
        ]}
        color={color}
      />

      <Rotor
        position={[
          0,
          0.08,
          1.45,
        ]}
        color={color}
      />

      <Rotor
        position={[
          0,
          0.08,
          -1.45,
        ]}
        color={color}
      />
    </group>
  );
}


function VehicleMarker({
  name,
  uav,
}) {
  if (
    uav.east == null ||
    uav.north == null
  ) {
    return null;
  }

  const altitude =
    Math.max(
      uav.altitude ?? 0,
      0,
    );

  const failed =
    uav.healthy === false;

  const color =
    failed
      ? "#d85f59"
      : (
        UAV_COLORS[name] ??
        "#c2ccd3"
      );

  return (
    <group>
      <Line
        points={[
          [
            uav.east,
            0,
            uav.north,
          ],
          [
            uav.east,
            altitude,
            uav.north,
          ],
        ]}
        color="#66747e"
        lineWidth={0.7}
        transparent
        opacity={0.35}
      />

      <group
        position={[
          uav.east,
          altitude,
          uav.north,
        ]}
        scale={0.72}
      >
        <Quadrotor
          color={color}
        />
      </group>

      <Html
        position={[
          uav.east,
          altitude + 2.3,
          uav.north,
        ]}
        center
        distanceFactor={19}
      >
        <div
          className={
            failed
              ? "uav-label failed"
              : "uav-label"
          }
        >
          <strong>
            {name}
          </strong>

          <span>
            {altitude.toFixed(
              1,
            )} m
          </span>
        </div>
      </Html>
    </group>
  );
}


function World({
  config,
  plan,
  uavs,
}) {
  const width =
    config?.width_m ?? 60;

  const height =
    config?.height_m ?? 30;

  const gridSize =
    Math.max(
      width,
      height,
      60,
    ) + 25;

  return (
    <>
      <color
        attach="background"
        args={[
          "#061018",
        ]}
      />

      <ambientLight
        intensity={1.25}
      />

      <directionalLight
        position={[
          width,
          45,
          height,
        ]}
        intensity={1.9}
      />

      <gridHelper
        args={[
          gridSize,
          Math.max(
            16,
            Math.round(
              gridSize / 5,
            ),
          ),
          "#1d3546",
          "#122633",
        ]}
        position={[
          width / 2,
          -0.06,
          height / 2,
        ]}
      />

      <SearchArea
        width={width}
        height={height}
      />

      {Object.entries(
        plan ?? {},
      ).map(
        ([name, segment]) => (
          <PlannedRoute
            key={name}
            name={name}
            segment={segment}
          />
        ),
      )}

      {Object.entries(
        uavs ?? {},
      ).map(
        ([name, uav]) => (
          <VehicleMarker
            key={name}
            name={name}
            uav={uav}
          />
        ),
      )}

      <OrbitControls
        makeDefault
        target={[
          width / 2,
          3,
          height / 2,
        ]}
        minDistance={12}
        maxDistance={160}
        maxPolarAngle={
          Math.PI / 2.03
        }
      />
    </>
  );
}


export default function MissionScene({
  config,
  plan,
  uavs,
}) {
  const width =
    config?.width_m ?? 60;

  const height =
    config?.height_m ?? 30;

  const distance =
    Math.max(
      width,
      height,
      45,
    );

  return (
    <div className="scene-shell">
      <Canvas
        camera={{
          position: [
            width * 0.65,
            distance * 0.68,
            height * 1.25,
          ],
          fov: 44,
          near: 0.1,
          far: 1000,
        }}
      >
        <World
          config={config}
          plan={plan}
          uavs={uavs}
        />
      </Canvas>

      <div className="scene-hint">
        drag · orbit · scroll
      </div>
    </div>
  );
}