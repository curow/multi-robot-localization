 python generate_route.py > cross.rou.xml
 duarouter --route-files=cross.rou.xml --net=cross.net.xml --output-file=cross.rou.xml
 sumo -n cross.net.xml -r cross.rou.xml --fcd-output fcd-output.xml
 python cross_visulization.py
