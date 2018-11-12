(* ::Package:: *)

(* ::Input::Initialization:: *)
BeginPackage["pathPlanning`"]
Unprotect@@Names["pathPlanning`*"];
ClearAll@@Names["pathPlanning`*"];

f::usage="f[x]"
importMaps::usage = "importMaps[]: Returns array of elements composed of the map in 2D matrix form and a list contaning the starting and ending points indicated in the README file. (ie: importMaps[][[1]]= {map1, {startPoint1, endPoint1}} )"
createGridGraph::usage = "createGridGraph[map]: Is given a 2D matrix of 1s (obstacles) and 0s (free space) and returns a graph object with a grid structure and the node coordinates as the node names (ie. VertexList[graph]= {{2.5, 3.5}, {2.5, 4.5}, {3.5, 3.5}, ...}"
builRRTGraph::usage= "builRRTGraph[region, origin, destination, density] Is given a 2D matrix of 1s (obstacles) and 0s (free space) and returns a graph object created with the RRT algorithm strarting from the origin. The final graph will have (Density*RegionArea) number of nodes"
createRegions::usage = "createRegions[map]: Is given a 2D matrix of 1s (obstacles) and 0s (free space) and returns a list with two elements. These are the discretized regions of the map (ie. {obstacleRegion, validRegion})"
dijkstra::usage = "dijkstra[graph, origin, destination]: Finds the shortest path from the origin node to the destination node using the Dijkstra algorithm"
aStar::usage = "aStar[graph, origin, destination]: Finds the shortest path from the origin node to the destination node using the A* algorithm"

Begin["`Private`"]

f[x_]:=Module[{},x^2];

importMaps[]:= Module[{},
mapdirs = Select[FileNames["*map*",NotebookDirectory[]],DirectoryQ];
maps = 
Table[
mapi = Import[ToString[mapdirs[[i]]]<>$PathnameSeparator<> "map" <> ToString[i]<>".csv"];
instructioni = ImportString[Import[ToString[mapdirs[[i]]]<>$PathnameSeparator<> "README.md"]];
objectivesi = StringCases[instructioni,{"Line "~~(l:DigitCharacter..) ->l, "Column "~~(l:DigitCharacter..) ->l}];
origini = ToExpression[objectivesi[[;;2]]];
destinationi = ToExpression[objectivesi[[3;;]]];
{mapi, {origini, destinationi}}
,
{i,  1, Length[mapdirs], 1}]
];

createGridGraph[map0_] := Module[{map=map0},
connections = Reap[
Table[
Table[
If[map[[i]][[j]] != 1 && map[[i-1]][[j]] != 1,  Sow[{i,j}-0.5\[UndirectedEdge]{i-1,j}-0.5]];
If[map[[i]][[j]] != 1 && map[[i+1]][[j]] != 1,  Sow[{i,j}-0.5\[UndirectedEdge]{i+1,j}-0.5]];
If[map[[i]][[j]] != 1 && map[[i]][[j-1]] != 1,  Sow[{i,j}-0.5\[UndirectedEdge]{i,j -1}-0.5]];
If[map[[i]][[j]] != 1 && map[[i]][[j+1]] != 1,  Sow[{i,j}-0.5\[UndirectedEdge]{i,j+1}-0.5]];
,{i, 2, Length[map]-1, 1}];
,{j, 2, Length[Transpose[map]]-1, 1}];
]//Last //Flatten;
connections= DeleteDuplicates[Sort/@connections];
validGrid = Graph[connections];
 mapGraph = Graph[connections, VertexCoordinates->VertexList[validGrid],  EdgeWeight->ConstantArray[1,Length[connections]]]
];

builRRTGraph[region0_, origin0_, destination0_, density0_] := Module[{validRegion=region0, origin= origin0, destination= destination0, density = density0},
qInit = origin  ; (* starting node *)
qEnd = destination;(* ending  node *)
kmax = RegionMeasure[validRegion]*density; (* max number of iterarions*)
\[CapitalDelta]q = 1.; (* increment distance for branches *)

qRand= RandomPoint[validRegion, 1][[1]];
qNew = qInit+ Normalize[qRand-qInit]*\[CapitalDelta]q;
 rrtGraph= Graph[{qInit\[UndirectedEdge]qNew}];

Table[
While[True,
qRand= RandomPoint[validRegion, 1][[1]];
qNear = Nearest[VertexList[rrtGraph], qRand][[1]];
qNew = qNear+ Normalize[qRand-qNear]*\[CapitalDelta]q;
If[RegionDistance[obstacleRegion, qNew]>\[CapitalDelta]q && RegionMember[validRegion, qNew],Break[]]
];

rrtGraph= EdgeAdd[rrtGraph, qNear\[UndirectedEdge]qNew];
,{k, 1, kmax, 1}];

(* Add the Destination node to the graph *)
i=0;
While[True,
i++;
qNear = Nearest[VertexList[rrtGraph], qEnd, i][[i]];
If[ToString[RegionIntersection[Line[{qNear,qEnd}],RegionBoundary[obstacleRegion]]] ==ToString[EmptyRegion[2]]||i>20,Break[]];
];

If[i>20, Print[Style["ERROR: could not join the destination to the graph, try with a greater density", Bold, Red, 20]]];

rrtGraph= EdgeAdd[rrtGraph, qNear\[UndirectedEdge]qEnd];

rrtGraph = Graph[EdgeList [rrtGraph], EdgeWeight->ConstantArray[\[CapitalDelta]q,Length[EdgeList[rrtGraph]]], VertexCoordinates->VertexList[rrtGraph]]
];

createRegions[map0_]:=Module[{map=map0},
regions = Reap[
Table[
Table[
If[map[[i]][[j]]== 1, Sow[Region[Rectangle[{i-1,j-1},{i,j}]], obstacle];, Sow[Region[Rectangle[{i-1,j-1},{i,j}], PlotTheme->"NoAxes"], valid];]
, {i, 1, Length[map]}]
,{j, 1, Length[Transpose[map]]}]
, {obstacle, valid}]//Last;
obstacleRegion = DiscretizeRegion[RegionUnion[regions[[1]]]];
validRegion = DiscretizeRegion[RegionUnion[regions[[2]]]];
{obstacleRegion,validRegion}
];

dijkstra[graph0_, origin0_, destination0_]:=Module[{mapGraph= graph0, origin= origin0, destination= destination0},
s = origin;
e = destination;
queue = {};
If[ToString[PropertyValue[{mapGraph,EdgeList[mapGraph][[1]]},EdgeWeight]]=="$Failed",(*if no edge weights assign edge weight = 1 *)
mapGraph = Graph[EdgeList [mapGraph], EdgeWeight->ConstantArray[1,Length[EdgeList[mapGraph]]], VertexCoordinates->VertexList[mapGraph]]
];
(* Create association lists*)
dijkstraData = 
Reap[
Sow[s ->0.,d];(*first node in queue*)
Table[
If[node!= s, 
Sow[node-> -1., d](*distancia*)
];
Sow[node-> Null, p];(*padre*)
Sow[node-> False,v]; (*visto*)
,{node, VertexList[mapGraph]}]
][[2]];

(*Turn association lists into 'dictionaries'*)
distance =<|dijkstraData[[1]]|>;
father = <|dijkstraData[[2]]|>;
seen = <|dijkstraData[[3]]|>;
AppendTo[queue,  {s, distance[s] }];

While[!seen[e],
queue =SortBy[queue, Last];
u=queue[[1]][[1]];
queue = Drop[queue, 1];
seen[u] = True;

Table[
If[distance[v] > distance[u]+PropertyValue[{mapGraph,u\[UndirectedEdge]v},EdgeWeight] || distance[v]==-1.,
distance[v]=distance[u]+PropertyValue[{mapGraph,u\[UndirectedEdge]v},EdgeWeight];
father[v]=u;
AppendTo[queue,  {v, distance[v]}];
]
,{v, AdjacencyList[mapGraph,u]}];
];

path =Last[Reap[
u = e;
Sow[u];
While[u!= s,
u=father[u];
Sow[u];
]
]][[1]]
];

fScore=Compile[{{nodexy, _Real, 1}, {destinationxy, _Real, 1}},
(*Sqrt[(ex-nx)^2+(ey-ny)^2]*)

Abs[(destinationxy[[1]]-nodexy[[1]])]+Abs[(destinationxy[[2]]-nodexy[[2]])]

,CompilationTarget->"C"];

aStar[graph0_, origin0_, destination0_]:=Module[{mapGraph= graph0, origin= origin0, destination= destination0},
s = origin;
e = destination;
queue = {};
If[ToString[PropertyValue[{mapGraph,EdgeList[mapGraph][[1]]},EdgeWeight]]=="$Failed",(*if no edge weights assign edge weight = 1 *)
mapGraph = Graph[EdgeList [mapGraph], EdgeWeight->ConstantArray[1,Length[EdgeList[mapGraph]]], VertexCoordinates->VertexList[mapGraph]]
];

(* Create association lists*)
aStarData = 
Reap[
Sow[s ->0.,d];(*first node in queue*)
Table[
If[node!= s, 
Sow[node-> -1., d](*distancia*)
];
Sow[node-> Null, p];(*padre*)
Sow[node-> False,v]; (*visto*)
Sow[node-> Null,s]; (*visto*)
,{node, VertexList[mapGraph]}]
][[2]];

(*Turn association lists into 'dictionaries'*)
distance =<|aStarData[[1]]|>;
father = <|aStarData[[2]]|>;
seen = <|aStarData[[3]]|>;
score = <|aStarData[[4]]|>;
AppendTo[queue,  {s, distance[s]}];
w=50.; (* weigth of the heuristic*)

While[!seen[e],
queue =SortBy[queue, Last];
u=queue[[1]][[1]];
queue = Drop[queue, 1];
seen[u] = True;

Table[
If[distance[v] > distance[u]+PropertyValue[{mapGraph,u\[UndirectedEdge]v},EdgeWeight] || distance[v]==-1,
distance[v]=distance[u]+PropertyValue[{mapGraph,u\[UndirectedEdge]v},EdgeWeight] ;
father[v]=u;
AppendTo[queue,  {v, distance[v]+w*fScore[v, e]}];
];
,{v, AdjacencyList[mapGraph,u]}];
];

path = 
Last[Reap[
u = e;
Sow[u];
While[u!= s,
u=father[u];
Sow[u];
]
]][[1]]
];





End[]
Protect@@Names["pathPlanning`*"];
EndPackage[]
