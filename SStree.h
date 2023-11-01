#ifndef SSTREE_H
#define SSTREE_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <queue>
#include <limits>
#include <fstream>
#include <numeric>

#include "params.h"
#include "Point.h"

auto compare = [](std::pair<NType, Point>& p1, std::pair<NType, Point>& p2) -> bool {
    return p1.first < p2.first;
};

class SsNode {
private:
    NType varianceAlongDirection(const std::vector<Point>& centroids, size_t direction) const {
        NType mean = std::accumulate( centroids.begin(), centroids.end(), static_cast<NType>(0.0), [direction](NType sum, const Point& point) { return sum + point[direction]; } ) / static_cast<NType>(centroids.size());
        NType variance = 0;
        for (auto& point:centroids) {
            variance += (point[direction] - mean) * (point[direction] - mean);
        }
        variance /= static_cast<NType>(centroids.size());
        return variance;
    };

    size_t minVarianceSplit(std::vector<Point>& centroids, size_t coordinateIndex){
        size_t min = Settings::m;
        size_t max = centroids.size()-min;

        size_t splitIndex = 0;
        NType minVariance = NType::max_value();

        for (size_t i = min; i <= max; i++) {
            std::vector<Point> left(centroids.begin(), centroids.begin() + i);
            std::vector<Point> right(centroids.begin() + i, centroids.end());

            NType sum_variance = varianceAlongDirection(left, coordinateIndex) + varianceAlongDirection(right, coordinateIndex);
            if (sum_variance < minVariance) {
                minVariance = sum_variance;
                splitIndex = i;
            }
        }
        return splitIndex;
    };
    
public:

    Point meanPoint(const std::vector<Point>& points) {
        return std::accumulate(
                points.begin(), points.end(),
                Point(points[0].dim()), [](Point sum, const Point& point) { return sum + point; }
                ) / static_cast<NType>(points.size());
    }

    std::pair<Point, Point> centroidsMBB(const std::vector<Point>& points) {
        size_t dims = points[0].dim();
        Point inf(dims);
        Point sup(dims);

        for (int dim = 0; dim < dims; ++dim) {
            NType min = NType::max_value();
            NType max = NType::min_value();

            for (const Point& p: points) {
                if (p[dim] < min) { min = p[dim]; }
                if (p[dim] > max) { max = p[dim]; }
            }
            inf[dim] = min;
            sup[dim] = max;
        }
        return std::make_pair(inf, sup);
    }

    SsNode(std::size_t dims): D(dims) {};
    virtual ~SsNode() = default;

    Point centroid;
    std::size_t D;
    NType radius;
    SsNode* parent = nullptr;

    virtual bool isLeaf() const = 0;

    virtual std::pair<NType, Point> bestCentroidAlongDirection(const Point& p1, const Point& p2, int n_iter) = 0;

    virtual std::vector<Point> getEntriesCentroids() const = 0;
    virtual void sortEntriesByCoordinate(size_t coordinateIndex) = 0;
    virtual std::pair<SsNode*, SsNode*> split() = 0;
    virtual bool intersectsPoint(const Point& point) const {
        return distance(this->centroid, point) <= this->radius;
    }

    virtual void updateBoundingEnvelope() = 0;
    size_t directionOfMaxVariance() const {
        NType maxVariance = 0;
        size_t directionIndex = 0;
        std::vector<Point> centroids = this->getEntriesCentroids();
        for (size_t i = 0; i < this->D; i++) {
            NType variance = this->varianceAlongDirection(centroids, i);
            if (variance > maxVariance) {
                maxVariance = variance;
                directionIndex = i;
            }
        }
        return directionIndex;
    };

    size_t findSplitIndex(){
        size_t maxVarianceDirectionIndex = directionOfMaxVariance();
        this->sortEntriesByCoordinate(maxVarianceDirectionIndex);
        std::vector<Point> centroids = this->getEntriesCentroids();
        return this->minVarianceSplit(centroids, maxVarianceDirectionIndex);
    };

    size_t dim() const { return centroid.dim(); }

//    virtual SsNode* insert(const Point& point) = 0;

    virtual std::pair<SsNode*, SsNode*> insert(const Point& point, const std::string& path) = 0;

    bool test(bool isRoot = false) const;
    void print(size_t indent = 0) const;

    //virtual void FNDFTrav(const Point& q, size_t k, std::priority_queue<Pair, std::vector<Pair>, Comparator>& L, NType& Dk) const = 0;

    virtual void saveToStream(std::ostream &out) const = 0;
    virtual void loadFromStream(std::istream &in, SsNode* parent) = 0;


    virtual void knn(const Point& center,
                     NType& radius,
                     size_t& k,
                     std::priority_queue<std::pair<NType, Point>, std::vector<std::pair<NType, Point>>, decltype(compare)>& nearest) = 0;

};

class SsInnerNode : public SsNode {
private:

    std::pair<Point, Point> hyperSpheresMBB(const std::vector<SsNode*>& nodes) {
        size_t dims = nodes[0]->centroid.dim();
        Point inf(dims);
        Point sup(dims);

        for (int dim = 0; dim < dims; ++dim) {
            NType min = NType::max_value();
            NType max = NType::min_value();

            for (SsNode* node : nodes) {
                NType current_min = node->centroid[dim] - node->radius;
                NType current_max = node->centroid[dim] + node->radius;

                if (current_min < min) { min = current_min; }
                if (current_max > max) { max = current_max; }
            }

            inf[dim] = min;
            sup[dim] = max;
        }
        return std::make_pair(inf, sup);
    };

    std::pair<Point, Point> hyperSpheresSecondMBB(std::pair<Point, Point>& mbb, const std::vector<SsNode*>& nodes) {
        size_t dims = nodes[0]->centroid.dim();
        Point inf(dims);
        Point sup(dims);

        for (int dim = 0; dim < dims; ++dim) {
            NType min = NType::max_value();
            NType max = NType::min_value();

            for (SsNode* node : nodes) {
                NType current_min = node->centroid[dim] - node->radius;
                NType current_max = node->centroid[dim] + node->radius;

                if (current_min < min && current_min > mbb.first[dim]) { min = current_min; }
                if (current_max < min && current_max > mbb.first[dim]) { min = current_max; }

                if (current_max > max && current_max < mbb.second[dim]) { max = current_max; }
                if (current_min > max && current_min < mbb.second[dim]) { max = current_min; }
            }

            if (min == NType::max_value()) { min = mbb.first[dim]; }
            if (max == NType::min_value()) { max = mbb.second[dim]; }

            inf[dim] = min;
            sup[dim] = max;
        }
        return std::make_pair(inf, sup);
    }

    std::pair<Point, Point> hyperSpheresIMBB(const std::vector<SsNode*>& nodes) {
        size_t dims = nodes[0]->centroid.dim();
        Point inf(dims);
        Point sup(dims);

        for (int dim = 0; dim < dims; ++dim) {
            NType min = NType::min_value();
            NType max = NType::max_value();

            for (SsNode* node : nodes) {
                NType current_min = node->centroid[dim] - node->radius;
                NType current_max = node->centroid[dim] + node->radius;

                if (current_min > min) { sup[dim] = current_min; }
                if (current_max < max) { inf[dim] = current_max; }
            }
        }

        return std::make_pair(inf, sup);
    }

    std::vector<Point> getEntriesCentroids() const override {
        std::vector<Point> result;
        for (auto& childNode:this->children) {
            result.emplace_back(childNode->centroid);
        }
        return result;
    };
    void sortEntriesByCoordinate(size_t coordinateIndex) override {
        std::sort(this->children.begin(), this->children.end(), [coordinateIndex](SsNode* a, SsNode* b) {
            return a->centroid[coordinateIndex] < b->centroid[coordinateIndex];
        });
    };

public:
    SsInnerNode(std::size_t dims): SsNode(dims) {};

    std::pair<SsNode*, SsNode*> split() override {
        size_t splitIndex = findSplitIndex();

        SsNode* newNode1 = new SsInnerNode(D);
        dynamic_cast<SsInnerNode*>(newNode1)->children = std::vector<SsNode*>(this->children.begin(), this->children.begin() + splitIndex);

        SsNode* newNode2 = new SsInnerNode(D);
        dynamic_cast<SsInnerNode*>(newNode2)->children = std::vector<SsNode*>(this->children.begin() + splitIndex, this->children.end());

        newNode1->updateBoundingEnvelope();
        newNode2->updateBoundingEnvelope();

        newNode1->parent = this->parent;
        newNode2->parent = this->parent;

        return std::make_pair(newNode1, newNode2);
    };

    std::vector<SsNode*> children;

    SsNode* findClosestChild(const Point& target) const {
        NType min_distance = NType::max_value();
        SsNode* closest_child = nullptr;

        for (SsNode* child : this->children) {
            NType dist = distance(child->centroid, target);
            if (dist < min_distance) {
                min_distance = dist;
                closest_child = child;
            }
        }
        return closest_child;
    };

    SsNode* findMinRadiusIncreaseChild(const Point& target) const {
        SsNode* closest = nullptr;

        NType min_radius_increase = NType::max_value();

        for (SsNode* node: this->children) {
            NType dist = distance(node->centroid, target);

            if (dist <= node->radius) { closest = node; break; }

            Point difference = target - node->centroid;
            Point direction = difference / difference.norm();

            Point bounding_point = node->centroid + direction * node->radius;

            NType radius_increase = distance(bounding_point, target);

            if (radius_increase < min_radius_increase) {
                min_radius_increase = radius_increase;
                closest = node;
            }
        }

        return closest;
    }

    bool isLeaf() const override { return false; }
    void updateBoundingEnvelope() override {
        std::vector<Point> centroids = getEntriesCentroids();

        std::pair<Point, Point> mbb = hyperSpheresMBB(children);
        std::pair<Point, Point> mbb2nd = hyperSpheresSecondMBB(mbb, children);
        std::pair<Point, Point> imbb = hyperSpheresIMBB(children);
        std::pair<Point, Point> mbb_centroids = centroidsMBB(centroids);

        Point mean = meanPoint(centroids);

        std::vector<Point> points = {
                mean,                                               // Promedio de todos los centroides
                (mbb_centroids.first + mbb_centroids.second) / 2.0, // Centro del MBB de centroides
                (mbb.first + mbb.second) / 2.0,                     // Centro del MBB de hiperesferas
                (imbb.first + imbb.second) / 2.0                    // Centro del MBB invertido
        };

        Point reference = meanPoint(points);
        points.push_back(reference);

        NType best_radius = NType::max_value();
        Point best_centroid;

        for (int i = 0; i < points.size(); ++i) {
            for (int j = i; j < points.size(); ++j) {
                if (distance(points[i], points[j]) > 0) {
                    std::pair<NType, Point> option = bestCentroidAlongDirection(points[i], points[j], 50);
                    if (option.first < best_radius) {
                        best_radius = option.first;
                        best_centroid = option.second;
                    }
                }
            }
        }

        if (distance(mbb2nd.first, mbb2nd.second) > 0) {
            std::pair<NType, Point> option = bestCentroidAlongDirection(mbb2nd.first, mbb2nd.second, 50);
            if (option.first < best_radius) {
                best_radius = option.first;
                best_centroid = option.second;
            }
        }

        this->radius = best_radius;
        this->centroid = best_centroid;
    };

    std::pair<NType, Point> bestCentroidAlongDirection(const Point& p1, const Point& p2, int n_iter) override {
        NType segment_distance = distance(p1, p2);
        NType step = segment_distance * 2.0 / (float) n_iter;

        Point difference = p2 - p1;
        Point direction = difference / difference.norm();

        Point best_centroid;
        Point current_centroid = p1 - direction * segment_distance / 2.0;

        NType current_centroid_radius;
        NType best_radius = NType::max_value();

        for (int i = 0; i < n_iter; ++i){
            current_centroid_radius = 0;

            for (SsNode* node: children) {
                NType dis = distance(node->centroid, current_centroid) + node->radius;
                if (dis > current_centroid_radius) {
                    current_centroid_radius = dis;
                }
            }

            if (current_centroid_radius < best_radius) {
                best_radius = current_centroid_radius;
                best_centroid = current_centroid;
            }

            current_centroid += direction * step;
        }

        return std::make_pair(best_radius, best_centroid);
    };

    void updateBoundingEnvelopeSplit() {
        std::vector<Point> centroids = getEntriesCentroids();

        std::pair<Point, Point> mbb = hyperSpheresMBB(this->children);
        std::pair<Point, Point> mbb2nd = hyperSpheresSecondMBB(mbb, this->children);
        std::pair<Point, Point> imbb = hyperSpheresIMBB(this->children);
        std::pair<Point, Point> mbb_centroids = centroidsMBB(centroids);

        Point mean = meanPoint(centroids);

        std::vector<Point> points = {
                mean,
                (mbb_centroids.first + mbb_centroids.second) / 2.0,
                (mbb.first + mbb.second) / 2.0,
                (imbb.first + imbb.second) / 2.0,
                (mbb2nd.first + mbb2nd.second) / 2.0
        };

        Point reference = meanPoint(points);
        points.push_back(reference);

        NType best_radius = NType::max_value();
        Point best_centroid;

        for (int i = 0; i < points.size(); ++i) {
            for (int j = i; j < points.size(); ++j) {
                if (distance(points[i], points[j]) > 0) {
                    std::pair<NType, Point> option = bestCentroidAlongDirection(points[i], points[j], 50);
                    if (option.first < best_radius) {
                        best_radius = option.first;
                        best_centroid = option.second;
                    }
                }
            }
        }

        if (distance(mbb2nd.first, mbb2nd.second) > 0) {
            std::pair<NType, Point> option = bestCentroidAlongDirection(mbb2nd.first, mbb2nd.second, 50);
            if (option.first < best_radius) {
                best_radius = option.first;
                best_centroid = option.second;
            }
        }

        if (children.size() > 2) {
            Point c1 = children[children.size() - 1]->centroid;
            NType r1 = children[children.size() - 1]->radius;

            Point c2 = children[children.size() - 2]->centroid;
            NType r2 = children[children.size() - 2]->radius;

            Point v = c1 - c2;
            Point dir = v / v.norm();

            Point lim1 = c1 + dir * r1;
            Point lim2 = c2 - dir * r2;

            Point center = (lim1 + lim2) / 2.0;

            std::vector<Point> newEntriesPoints = {
                    c1, c2, (c1 + c2) / 2.0, center
            };

            for (Point &point: newEntriesPoints) {
                if (distance(centroid, point) > 0) {
                    std::pair<NType, Point> option = bestCentroidAlongDirection(centroid, point, 50);
                    if (option.first < best_radius) {
                        best_radius = option.first;
                        best_centroid = option.second;
                    }
                }
            }
        }

        this->radius = best_radius;
        this->centroid = best_centroid;
    };

    std::pair<SsNode*, SsNode*> insert(const Point& point, const std::string& path) override {
        SsNode* closestChild = findMinRadiusIncreaseChild(point);

        std::pair<SsNode*, SsNode*> newChildren = closestChild->insert(point, path);
        SsNode* newChild1 = newChildren.first;
        SsNode* newChild2 = newChildren.second;

        if (newChild1 == nullptr) {
            children.erase(std::remove(children.begin(),children.end(),closestChild),children.end());
            children.emplace_back(closestChild);
            updateBoundingEnvelope();
            return std::make_pair(nullptr, nullptr);
        } else {
            children.erase(std::remove(children.begin(),children.end(),closestChild),children.end());

            children.emplace_back(newChild1);
            children.emplace_back(newChild2);
            updateBoundingEnvelopeSplit();

            if (children.size() <= Settings::M) {
                return std::make_pair(nullptr, nullptr);
            }
        }

        return split();
    };

    // void FNDFTrav(const Point& q, size_t k, std::priority_queue<Pair, std::vector<Pair>, Comparator>& L, NType& Dk) const override;

    virtual void saveToStream(std::ostream &out) const override;
    virtual void loadFromStream(std::istream &in, SsNode* parent) override;

    void knn(const Point& center,
             NType& radius,
             size_t& k,
             std::priority_queue<std::pair<NType, Point>, std::vector<std::pair<NType, Point>>, decltype(compare)>& nearest) override;

};

class SsLeaf : public SsNode {
private:

    std::vector<Point> getEntriesCentroids() const override {
        return this->points;
    };

    void sortEntriesByCoordinate(size_t coordinateIndex) override {
        std::vector<std::pair<Point, std::string>> entries;
        for (size_t i = 0; i < this->points.size(); i++) {
            entries.emplace_back(std::make_pair(this->points[i], this->paths[i]));
        }
        std::sort(entries.begin(), entries.end(), [coordinateIndex](std::pair<Point, std::string>& a, std::pair<Point, std::string>& b) { return a.first[coordinateIndex] < b.first[coordinateIndex]; });

        this->points = std::vector<Point>(entries.size());
        this->paths = std::vector<std::string>(entries.size());

        for (size_t i = 0; i < entries.size(); i++) {
            this->points[i] = entries[i].first;
            this->paths[i] = entries[i].second;
        }
    };

public:

    SsLeaf(std::size_t dims): SsNode(dims) {};

    std::pair<SsNode*, SsNode*> split() override {
        size_t splitIndex = findSplitIndex();

        auto* newNode1 = new SsLeaf(D);
        newNode1->parent = parent;

        for (size_t i = 0; i < splitIndex; ++i) {
            newNode1->insert(points[i], paths[i]);
        }

        auto* newNode2 = new SsLeaf(D);
        newNode2->parent = parent;

        for (size_t i = splitIndex; i < points.size(); ++i) {
            newNode2->insert(points[i], paths[i]);
        }

        newNode1->updateBoundingEnvelope();
        newNode2->updateBoundingEnvelope();

        return std::make_pair(newNode1, newNode2);
    };


    std::vector<std::string> paths;
    std::vector<Point> points;

    bool isLeaf() const override { return true; }

    std::pair<NType, Point> kFarthestNeighbours(int k) {
        Point best_centroid = points[0];
        NType best_radius = NType::max_value();

        for (int i = 0; i < points.size(); ++i) {
            Point target = points[i];
            std::priority_queue<Point, std::vector<Point>, std::function<bool(Point&, Point&)>> pq {
                    [&](Point& p1, Point& p2) -> bool {
                        return distance(p1, target) > distance(p2, target);
                    }};

            for (int j = 0; j < points.size(); ++j) {
                if (j == i) {
                    continue;
                }
                if (pq.size() < k) {
                    pq.push(points[j]);
                } else if (distance(points[j], target) > distance(pq.top(), target)) {
                    pq.pop();
                    pq.push(points[j]);
                }
            }

            while (!pq.empty()) {
                Point farther = pq.top();
                pq.pop();
                Point candidate_centroid = (target + farther) / 2.0;
                NType candidate_radius = distance(candidate_centroid, farther);
                for (Point& p: points) {
                    NType dis = distance(candidate_centroid, p);
                    if (dis > candidate_radius) {
                        candidate_radius = dis;
                    }
                }

                if (candidate_radius < best_radius) {
                    best_radius = candidate_radius;
                    best_centroid = candidate_centroid;
                }
            }
        }

        return std::make_pair(best_radius, best_centroid);
    }

    std::pair<NType, Point> meanShift() {
        Point mean = meanPoint(getEntriesCentroids());
        return bestCentroidAlongDirection(centroid, mean, 50);
    }

    std::pair<NType, Point> bestCentroidAlongDirection(const Point& p1, const Point& p2, int n_iter) override {
        NType segment_distance = distance(p1, p2);
        NType step = segment_distance / (float) n_iter;

        Point difference = p2 - p1;
        Point unit_vector = difference / difference.norm();

        Point best_centroid;
        Point current_centroid = p1;

        NType current_centroid_radius;
        NType best_radius = NType::max_value();

        for (int i = 0; i < n_iter; ++i){
            current_centroid_radius = 0;

            for (Point& point: points) {
                NType dis = distance(point, current_centroid);
                if (dis > current_centroid_radius) {
                    current_centroid_radius = dis;
                }
            }

            if (current_centroid_radius < best_radius) {
                best_radius = current_centroid_radius;
                best_centroid = current_centroid;
            }

            current_centroid += unit_vector * step;

        }

        return std::make_pair(best_radius, best_centroid);
    }

    void updateBoundingEnvelope() override {
        if (points.size() == 1) {
            this->radius = 0;
            this->centroid = points[0];
            return;
        } else if (points.size() == 2) {
            this->centroid = (points[0] + points[1]) / 2.0;
            this->radius = distance(points[0], centroid);
            return;
        }

        if (distance(points.back(), centroid) <= radius) {
            return;
        }

        int k = std::ceil(0.4f * (float) points.size());

        std::vector<std::pair<NType, Point>> options {
                kFarthestNeighbours(k),
                meanShift(),
                bestCentroidAlongDirection(points.back(), meanPoint(points), 50),
                bestCentroidAlongDirection(points.back(), meanPoint(std::vector<Point> {points.begin(), points.end() - 1}), 50),
        };

        NType best_radius = NType::max_value();
        Point best_centroid;

        for (std::pair<NType, Point>& option: options) {
            if (option.first < best_radius) {
                best_radius = option.first;
                best_centroid = option.second;
            }
        }

        this->centroid = best_centroid;
        this->radius = best_radius;
    };

    std::pair<SsNode*, SsNode*> insert(const Point& point, const std::string& path) override {
        if (std::find(points.begin(), points.end(), point) != points.end()) {
            return std::make_pair(nullptr, nullptr);
        }

        this->points.emplace_back(point);
        this->paths.emplace_back(path);
        updateBoundingEnvelope();

        if (this->points.size() <= Settings::M) {
            return std::make_pair(nullptr, nullptr);
        }

        return split();
    };

   // void FNDFTrav(const Point& q, size_t k, std::priority_queue<Pair, std::vector<Pair>, Comparator>& L, NType& Dk) const override;

    virtual void saveToStream(std::ostream &out) const override;
    virtual void loadFromStream(std::istream &in, SsNode* parent) override;

    void knn(const Point& center,
             NType& radius,
             size_t& k,
             std::priority_queue<std::pair<NType, Point>, std::vector<std::pair<NType, Point>>, decltype(compare)>& nearest) override;


};


struct Pair {
    Point point;
    NType distance;

    Pair(const Point& p, NType d) : point(p), distance(d) {}
};
struct Comparator {
    bool operator()(const Pair& a, const Pair& b) const {
        return a.distance < b.distance; // max-heap basado en distancia
    }
};

class SsTree {
private:
    std::size_t D;
    SsNode* root;

    SsNode* search(SsNode* node, const Point& target);
    SsNode* searchParentLeaf(SsNode* node, const Point& target);

    void _kNNQuery(const Point &target, size_t k, SsNode *node, NType radius, std::priority_queue<std::pair<NType, Point>, std::vector<std::pair<NType, Point>>, decltype(compare)> &result) const ;

public:
    SsTree(std::size_t _D) : root(nullptr), D(_D) {}
    ~SsTree() {
        delete root;
    }
    
    void insert(const Point& point){};

    void insert(const Point& point, const std::string& path){
        if (this->root == nullptr) {
            this->root = new SsLeaf(D);

            this->root->parent = nullptr;
            this->root->centroid = point;
            this->root->radius = 0;
        }

        std::pair<SsNode*, SsNode*> newChildren = this->root->insert(point, path);
        SsNode* newChild1 = newChildren.first;
        SsNode* newChild2 = newChildren.second;

        if (newChild1 == nullptr) { return; }

        this->root = new SsInnerNode(D);
        this->root->parent = nullptr;

        newChild1->parent = root;
        newChild2->parent = root;

        dynamic_cast<SsInnerNode*>(this->root)->children.emplace_back(newChild1);
        dynamic_cast<SsInnerNode*>(this->root)->children.emplace_back(newChild2);

        (dynamic_cast<SsInnerNode*>(this->root))->updateBoundingEnvelopeSplit();
    };

//    void build (const std::vector<Point>& points);
    std::vector<Point> kNNQuery(const Point& center, size_t k) const;

    void print() const;
    void test() const;

    void saveToFile(const std::string &filename) const;
    void loadFromFile(const std::string &filename);

    std::vector<Point> linearKNN(const Point& center, size_t k) const {

    }
};




#endif // !SSTREE_H