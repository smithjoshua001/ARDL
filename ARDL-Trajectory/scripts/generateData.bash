#!/usr/bin/env sh
echo $1
model=${1}
mkdir ${model}_cubes
mkdir ${model}_cubes/Constraints
${2} 50000 ${model}/model.urdf ../ARDL-Trajectory/src/IdentificationTrajectory/template.json ./${model}_cubes/Constraints/
for i in {0..8}; do
mkdir ${model}_cubes/cube${i}Trajectories
for j in {0..7}; do
${3} ../ARDL-Trajectory/src/IdentificationTrajectory/optim.json kuka-lwr-4plus/model.urdf ./${model}_cubes/Constraints/Contraints_${i}.json ./${model}_cubes/cube${i}Trajectories/${j} 5 5
done
done
