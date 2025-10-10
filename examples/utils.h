#ifndef EXAMPLES_UTILS_H_
#define EXAMPLES_UTILS_H_

#include <fstream>
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

template <typename PointT, typename ContainerT>
void readPoints(const std::string& filename, ContainerT& points)
{
  std::ifstream in(filename.c_str());
  std::string line;
  boost::char_separator<char> sep(" ");
  // read point cloud from "freiburg format"
  while (!in.eof())
  {
    std::getline(in, line);
    in.peek();

    boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
    std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());

    if (tokens.size() != 6) continue;
    float x = boost::lexical_cast<float>(tokens[3]);
    float y = boost::lexical_cast<float>(tokens[4]);
    float z = boost::lexical_cast<float>(tokens[5]);

    points.push_back(PointT(x, y, z));
  }

  in.close();
}

template <typename PointT, typename ContainerT>
void readPoints_txt(const std::string& filename, ContainerT& points)
{
  std::ifstream in(filename.c_str());
  std::string line;
  boost::char_separator<char> sep(" ");
  while (!in.eof())
  {
    std::getline(in, line);
    in.peek();

    boost::tokenizer<boost::char_separator<char>> tokenizer(line, sep);
    std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());

    if (tokens.size() != 3) continue;
    float x = boost::lexical_cast<float>(tokens[0]);
    float y = boost::lexical_cast<float>(tokens[1]);
    float z = boost::lexical_cast<float>(tokens[2]);

    points.push_back(PointT(x, y, z));

  }
  in.close();
}


template <typename PointT, typename ContainerT>
void writePoints(const std::string& filename, const ContainerT& points)
{
    std::ofstream out(filename.c_str());
    if (!out.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }

    // Write PLY header
    out << "ply" << std::endl;
    out << "format ascii 1.0" << std::endl;
    out << "element vertex " << points.size() << std::endl;
    out << "property float x" << std::endl;
    out << "property float y" << std::endl;
    out << "property float z" << std::endl;
    out << "end_header" << std::endl;

    // Write points
    for (const auto& point : points) {
        out << point.x << " " << point.y << " " << point.z << std::endl;
    }

    out.close();
}


#endif /* EXAMPLES_UTILS_H_ */
