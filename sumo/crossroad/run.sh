 python generate_flow.py > cross.flow.xml
 duarouter --route-files=cross.flow.xml --net=cross.net.xml --output-file=cross.rou.xml
 sumo -n cross.net.xml -r cross.rou.xml --fcd-output fcd-output.xml
 python cross_visulization.py
