#include "SStree.h"


bool SsNode::test(bool isRoot) const {
    size_t count = 0;

    if (this->isLeaf()) {
        const SsLeaf* leaf = dynamic_cast<const SsLeaf*>(this);
        count = leaf->points.size();

        // Verificar si los puntos están dentro del radio del nodo
        for (const Point& point : leaf->points) {
            if (distance(this->centroid, point) > this->radius) {
                std::cout << "Point outside node radius detected." << std::endl;
                return false;
            }
        }
    } else {
        const SsInnerNode* inner = dynamic_cast<const SsInnerNode*>(this);
        count = inner->children.size();

        // Verificar si los centroides de los hijos están dentro del radio del nodo padre
        for (const SsNode* child : inner->children) {
            if (distance(this->centroid, child->centroid) > this->radius) {
                std::cout << "Child centroid outside parent radius detected." << std::endl;
                return false;
            }
            // Verificar recursivamente cada hijo
            if (!child->test(false)) {
                return false;
            }
        }
    }

    // Comprobar la validez de la cantidad de hijos/puntos
    if (!isRoot && (count < Settings::m || count > Settings::M)) {
        std::cout << "Invalid number of children/points detected." << std::endl;
        return false;
    }

    // Comprobar punteros de parentezco, salvo para el nodo raíz
    if (!isRoot && !parent) {
        std::cout << "Node without parent detected." << std::endl;
        return false;
    }

    return true;
}

void SsTree::test() const {
    if (!root) {
        std::cout << "SS-Tree is empty!" << std::endl;
    }

    bool result = root->test(true);

    if (root->parent) {
        std::cout << "Root node parent pointer is not null!" << std::endl;
        result = false;
    }

    if (result) {
        std::cout << "SS-Tree is valid!" << std::endl;
    } else {
        std::cout << "SS-Tree has issues!" << std::endl;
    }
}

void SsNode::print(size_t indent) const {
    for (size_t i = 0; i < indent; ++i) {
        std::cout << "  ";
    }

    // Imprime información del nodo.
    std::cout << "Radius: " << radius;
    if (isLeaf()) {
//        const SsLeaf* leaf = dynamic_cast<const SsLeaf*>(this);
//        std::cout << std::endl;
//        std::cout << ", Points: [ ";
//        std::cout << "]";
    } else {
        std::cout << std::endl;
        const SsInnerNode* inner = dynamic_cast<const SsInnerNode*>(this);
        for (const SsNode* child : inner->children) {
            child->print(indent + 1);
        }
    }
    std::cout << std::endl;
}

void SsTree::print() const {
    if (root) {
        std::cout << "Root: " << root->radius << std::endl;

        for (const SsNode* child : static_cast<const SsInnerNode*>(root)->children) {
            std::cout<< "Child: " << child->radius << std::endl;
        }
//        root->print();
    } else {
        std::cout << "Empty tree." << std::endl;
    }
}

void SsLeaf::saveToStream(std::ostream &out) const {
    // Guardar centroid
    centroid.saveToFile(out, D);

    // Guardar el radio
    float radius_ = radius.getValue();
    out.write(reinterpret_cast<const char*>(&radius_), sizeof(radius_));

    // Guardar el numero de puntos
    size_t numPoints = points.size();
    out.write(reinterpret_cast<const char*>(&numPoints), sizeof(numPoints));

    // Guardar los puntos
    for (const auto& point : points) {
        point.saveToFile(out, D);
    }

    // Guardar las rutas (paths)
    size_t numPaths = paths.size();
    out.write(reinterpret_cast<const char*>(&numPaths), sizeof(numPaths));
    for (const auto& p : paths) {
        size_t pathLength = p.size();
        out.write(reinterpret_cast<const char*>(&pathLength), sizeof(pathLength));
        out.write(p.c_str(), (long) pathLength);
    }
}

void SsInnerNode::saveToStream(std::ostream &out) const {
    // Guardar centroid
    centroid.saveToFile(out, D);

    // Guardar el radio
    float radius_ = radius.getValue();
    out.write(reinterpret_cast<const char*>(&radius_), sizeof(radius_));

    // Guardar si apunta a nodos hoja
    bool pointsToLeafs = children[0]->isLeaf();
    out.write(reinterpret_cast<const char*>(&pointsToLeafs), sizeof(pointsToLeafs));

    // Guardar la cantidad de hijos para saber cuántos nodos leer después
    size_t numChildren = children.size();
    out.write(reinterpret_cast<const char*>(&numChildren), sizeof(numChildren));

    // Guardar los hijos
    for (const auto& child : children) {
        child->saveToStream(out);
    }
}

void SsInnerNode::loadFromStream(std::istream &in, SsNode* parent) {
    this->parent = parent;

    // Leer centroid
    centroid.readFromFile(in, D);

    // leer el valor del radio
    float radius_ = 0;
    in.read(reinterpret_cast<char*>(&radius_), sizeof(radius_));
    this->radius = radius_;

    // leer si apunta a hojas o nodos internos
    bool pointsToLeaf = false;
    in.read(reinterpret_cast<char*>(&pointsToLeaf), sizeof(pointsToLeaf));

    // leer cantidad de hijos
    size_t numChildren;
    in.read(reinterpret_cast<char*>(&numChildren), sizeof(numChildren));

    // leer hijos
    for (size_t i = 0; i < numChildren; ++i) {
        SsNode* child = pointsToLeaf ? static_cast<SsNode*>(new SsLeaf(D)) : static_cast<SsNode*>(new SsInnerNode(D));
        child->loadFromStream(in, this);
        children.push_back(child);
    }
}

void SsLeaf::loadFromStream(std::istream &in,  SsNode* parent) {
    this->parent = parent;

    // Leer centroid
    centroid.readFromFile(in, D);

    // Leer radio
    float radius_ = 0;
    in.read(reinterpret_cast<char*>(&radius_), sizeof(radius_));
    this->radius = radius_;

    // Leer numero de puntos
    size_t numPoints;
    in.read(reinterpret_cast<char*>(&numPoints), sizeof(numPoints));

    // Leer puntos
    points.resize(numPoints);
    for (size_t i = 0; i < numPoints; ++i) {
        points[i].readFromFile(in, D);
    }

    // Leer rutas (paths)
    size_t numPaths;
    in.read(reinterpret_cast<char*>(&numPaths), sizeof(numPaths));
    paths.resize(numPaths);
    for (size_t i = 0; i < numPaths; ++i) {
        size_t pathLength;
        in.read(reinterpret_cast<char*>(&pathLength), sizeof(pathLength));
        char* buffer = new char[pathLength + 1];
        in.read(buffer, (long) pathLength);
        buffer[pathLength] = '\0';
        paths[i] = std::string(buffer);
        delete[] buffer;
    }
}

void SsTree::saveToFile(const std::string &filename) const {
    std::ofstream out(filename, std::ios::binary);
    if (!out) {
        throw std::runtime_error("Cannot open file for writing");
    }

    // Guardar las dimensiones de la estructura
    out.write(reinterpret_cast<const char*>(&D), sizeof(D));

    // Guardar si el root es hija o nodo interno
    bool isLeaf = root->isLeaf();
    out.write(reinterpret_cast<const char*>(&isLeaf), sizeof(isLeaf));

    // Guardar el resto de la estructura
    root->saveToStream(out);
    out.close();
}

void SsTree::loadFromFile(const std::string &filename) {
    std::ifstream in(filename, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Cannot open file for reading");
    }
    if (root) {
        delete root;
        root = nullptr;
    }

    // Aquí se asume que el primer valor determina las dimensiones
    in.read(reinterpret_cast<char*>(&D), sizeof(D));

    // El segundo valor determina si el root es hoja
    bool isLeaf;
    in.read(reinterpret_cast<char*>(&isLeaf), sizeof(isLeaf));
    if (isLeaf) {
        root = new SsLeaf(D);
    } else {
        root = new SsInnerNode(D);
    }
    root->loadFromStream(in, nullptr);
    in.close();
}


void SsInnerNode::knn(const Point& q,
                      NType& D_k,
                      size_t& k,
                      std::priority_queue<std::pair<NType, Point>, std::vector<std::pair<NType, Point>>, decltype(compare)>& nearest) {

    NType r_max = this->radius;
    auto M_p = this->centroid;

    NType r_min = NType::max_value();

    if ((D_k + r_max) < distance(q, M_p)){

    } else if ( D_k + distance(q, M_p) < r_min) {

    }

    for (auto& node: this->children) {
        auto dist = distance(node->centroid, M_p) - node->radius;
        if (dist < r_min) {
            node->knn(q, D_k, k, nearest);
        }
    }
}

void SsLeaf::knn(const Point& q,
                 NType& D_k,
                 size_t& k,
                 std::priority_queue<std::pair<NType, Point>, std::vector<std::pair<NType, Point>>, decltype(compare)>& nearest) {

    NType r_max = this->radius;
    NType r_min = NType::max_value();
    auto M_p = this->centroid;

    for (auto& point: this->points) {
        auto dist = distance(this->centroid, point);
        if (dist < r_min) {
            r_min = dist;
        }
    }

    for (auto& o:this->points) {

        if ( (D_k + distance(o, M_p)) < distance(q, M_p) ) {
            // No sirve
            continue;
        } else if (D_k + distance(q, M_p) < distance(o, M_p)) {
            // No sirve
            continue;
        } else {
            // Sirve
            auto dist = distance(o, q);
            if (dist < D_k) {
                nearest.emplace(dist, o);
                if (nearest.size() > k) {
                    nearest.pop();
                }
                D_k = nearest.top().first;
            }
        }
    }
}

std::vector<Point> SsTree::kNNQuery(const Point &center, size_t k) const {
    if (this->root == nullptr) { return std::vector<Point>(); }

    std::vector<Point> points;
    NType radius = NType::max_value();
    std::priority_queue<std::pair<NType, Point>, std::vector<std::pair<NType, Point>>, decltype(compare)> nearest(compare);

//    for (int i = 0; i < k; ++i) {
//        nearest.emplace(radius, Point(this->D));
//    }

    this->root->knn(center, radius, k, nearest);

    std::cout<<"nearest size:"<<nearest.size()<<std::endl;
    for (int i = 0; i < k; i++) {
        if (nearest.empty()) { break; }
        points.push_back(nearest.top().second);
        std::cout<<"Distancia:"<<nearest.top().first<<std::endl;
        nearest.pop();
    }

    std::vector<Point> final_result;
    std::reverse_copy(points.begin(), points.end(), std::back_inserter(final_result));
    return final_result;
};