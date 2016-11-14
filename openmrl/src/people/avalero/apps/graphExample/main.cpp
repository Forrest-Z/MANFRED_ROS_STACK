#include "../../../../libraries/multiplatform/graphs/graphtypes.h"

#include "mrcore.h"
#include <iostream>
#include <ctime>
#include <cstdlib>
#include <climits>

using namespace mr;
using namespace std;

SimpleGraph myGraph;
vector<SimpleNode*> nodes;
vector<SimpleEdge*> edges;
SimpleGraphSearcher * searcher = NULL;

void randomGraph(){

	myGraph.clear();
	nodes.clear();
	edges.clear();

	int a;
	cout << "Introduce en numero de Nodos: ";
	cin >> a;

	srand((unsigned)time(0));
	int lowest=0, highest=a-1;
	double range=(highest-lowest)+1;

	for(int index=0; index<a; index++){
		nodes.push_back(myGraph.addNode(index));
	}

	int b;
	cout << "Introduce en numero de Arcos: ";
	cin >> b;

	srand((unsigned)time(0));
	int random_integer;
	for(int index=0; index<b; index++){
		int node1 = (int)(lowest+range*rand()/(RAND_MAX));
		int node2 = (int)(lowest+range*rand()/(RAND_MAX));

		if (node1>nodes.size()) node1=nodes.size()-1;
		if (node2>nodes.size()) node2=nodes.size()-1;

		if (!myGraph.isAdjacent(nodes[node1],nodes[node2]))
			edges.push_back(myGraph.addEdge(nodes[node1],nodes[node2]));
		else
			index--;
	}
}

void addNodes(){
	int a;
	cout << "Introduce en numero de Nodos: ";
	cin >> a;

	for (int i=0;i<a;i++){
		cout<<"Nodo "<<i<<":"<<endl;
		int value;
		cout << "value: "; cin >> value;
		nodes.push_back(myGraph.addNode(value));
		cout << "Añadido Nodo "<< nodes.size() << endl;
	}

	//myGraph.printfNodes();
}

void addArcs(){
	int a;
	cout << "Introduce en numero de Arcos: ";
	cin >> a;

	for (int i=0;i<a;i++){
		cout<<"Arco "<<i<<":"<<endl;
		int nodo1, nodo2;
		cout << "Primer Nodo: "; cin >> nodo1;
		cout << "Segundo Nodo: "; cin >> nodo2;
		edges.push_back(myGraph.addEdge(nodes[nodo1],nodes[nodo2]));
	}


	//myGraph.printfEdges();
}


void save(){
	string fileName;
	cout<<"Introduce el nombre del archivo: ";
	cin>>fileName;

	myGraph.saveToFile(fileName);
}

void load(){
	string fileName;
	cout<<"Introduce el nombre del archivo: ";
	cin>>fileName;

//	myGraph.loadFromFile(fileName);

}

void performSearch(){

	//myGraph.printfNodes();
	cout << "Search Algorithm: "<<endl;
	cout << "0. Breadth-First." <<endl;
	cout << "1. Depth-First." <<endl;
	cout << "2. Dijkstra Search" << endl;
	cout << "3. A-Star Search" << endl;

	int alg;

	cin >> alg;
/*
	cout << "Bidirectional Search?: "<<endl;
	cout << "0. No." <<endl;
	cout << "1. Yes." <<endl;

	bool bidi;

	cin >> bidi;
*/
	int initNode, finalNode;
	cout << "Init Node: ";
	cin >> initNode;
	cout << "Goal Node: ";
	cin >> finalNode;
	mr::MRTime timer;
	timer.precistic();

	vector<SimpleNode*> path;

	cout << "Nodes: " << *nodes[initNode] << " " << *nodes[finalNode] << endl;
	switch(alg){
		case 0:
			searcher = new UninformedSimpleGraphSearcher(&myGraph);
			((UninformedSimpleGraphSearcher*)searcher)->setAlgorithm(1);
			break;
		case 1:
			searcher = new UninformedSimpleGraphSearcher(&myGraph);
			((UninformedSimpleGraphSearcher*)searcher)->setAlgorithm(0);
			break;
		case 2:
			searcher = new DijkstraSimpleGraphSearcher(&myGraph,0,INT_MAX);
			break;
		case 3:
			searcher = new AstarSimpleGraphSearcher(&myGraph,0,INT_MAX);
			break;
	}


	if(searcher->findPath(nodes[initNode],nodes[finalNode])){
		cout << "Time ellapsed " << timer.precistoc() << endl;

		cout << "Path found"<<endl;
		path=searcher->getPath();

		//PRINT
		cout << "Path: ";
		for (int i=0;i<path.size();i++){
		  cout << *(path[i]) << " ";
		}
		cout << endl;
	}else{
		cout << "Time ellapsed " << timer.precistoc() << endl;
		cout << "No path found"<<endl;
	}

}

int menu(){
	int a;
	cout<<"Escoja Opcion"<<endl;
	cout<<"---------------------"<<endl;
	cout<<"1. Crear Nuevo Grafo"<<endl;
	cout<<"2. Borrar Nodo"<<endl;
	cout<<"3. Borrar Arco"<<endl;
	cout<<"4. Añadir Nodos"<<endl;
	cout<<"5. Añadir Arcos"<<endl;
	cout<<"6. Vaciar el grafo"<<endl;
	cout<<"---------------------"<<endl;

	cout<<"7. Imprimir Informacion del Grafo"<<endl;
	cout<<"---------------------"<<endl;

	cout<<"8. Realizar Busqueda"<<endl;
	cout<<"---------------------"<<endl;

	cout<<"9. Salvar Grafo en Archivo"<<endl;
	cout<<"10. Cargar Grafo de Archivo"<<endl;
	cout<<"---------------------"<<endl;
	cout<<"11. Crear grafo aleatorio"<<endl;

	cout<<"20. Salir"<<endl;
	cin >> a;

	switch(a){
		case 1:
			myGraph.clear();
			nodes.clear();
			edges.clear();
			addNodes();
			addArcs();
			myGraph.computeEdgesWeight();
			break;
		case 2:
			//removeNode();
			break;
		case 3:
			//removeArc();
			break;
		case 4:
			addNodes();
			myGraph.computeEdgesWeight();
			break;
		case 5:
			addArcs();
			myGraph.computeEdgesWeight();
			break;
		case 6:
			myGraph.clear();
			nodes.clear();
			edges.clear();
			break;
		case 7:
			cout << myGraph << endl;
			break;
		case 8:
		  performSearch();
		  break;
		case 9:
			save();
			break;
		case 10:
			load();
			break;
		case 11:
			randomGraph();
			myGraph.computeEdgesWeight();
			break;
		case 20:
			cout<<"Adios. Gracias!"<<endl;
			if (searcher!=NULL) delete searcher;
			break;
		}


	return a;
}




int main(void){


	while(menu()!=20){;}

	return 0;
}

