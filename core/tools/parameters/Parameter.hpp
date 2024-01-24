//
// Created by root on 12/21/23.
//

#ifndef NAV24_PARAMETER_HPP
#define NAV24_PARAMETER_HPP

#include <memory>
#include <string>
#include <vector>
#include <map>

#include <opencv2/core/core.hpp>


namespace NAV24 {

    class Parameter;
    typedef std::shared_ptr<Parameter> ParamPtr;
    class Parameter {
    public:
        // todo: can change cv::FileStorage to file name
        // Writes the internal data to a target (specified by key) in the file storage
        virtual void write(const std::string& key, cv::FileNode& write_node) const = 0;
        // Reads a node corresponding to a key (or array element)
        // and returns a parameter holding that node
        virtual ParamPtr read(const std::string& key) = 0;

        // Helper methods for key management
        static void splitKey(std::vector<std::string> &vKeys, const std::string& key, const std::string& delim = "/");
        static std::string& mergeKey(const std::vector<std::string>& vKey, const std::string& delim = "/");

        // Prints the node recursively from root to the last child
        virtual std::string printStr(const std::string& prefix = "") = 0;

    protected:
        static cv::FileNode* seekNode(const cv::FileNode& startNode, const std::string& key);
    };

    class ParamCV : public Parameter {
    public:
        ParamCV() = default;
        explicit ParamCV(const cv::FileNode& node) : mFileNode(node) {}

        virtual void write(const std::string& key, cv::FileNode& write_node) const override;
        virtual ParamPtr read(const std::string& key) override;

        std::string printStr(const std::string& prefix = "") override;

        cv::FileNode& getNode() { return mFileNode; }
        void setNode(const cv::FileNode& node) { mFileNode = node; }

    private:
        cv::FileNode mFileNode;
    };

    template<typename T>
    class ParamSeq : public ParamCV {
    public:
        enum SeqType {
            DEFAULT,
            BOOL,
            INT,
            STRING,
            FLOAT,
            DOUBLE
        } mType;

        ParamSeq<T>() : mType(SeqType::DEFAULT) {}
        explicit ParamSeq<T>(const SeqType& st) : mType(st) {}

        std::string printStr(const std::string &prefix) override;

        void push_back(T data) { mData.push_back(data); }
        T& get(int idx) { return mData[idx]; }

    private:
        std::vector<T> mData;
    };
}

#endif //NAV24_PARAMETER_HPP
