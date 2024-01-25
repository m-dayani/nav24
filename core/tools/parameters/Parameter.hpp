//
// Created by root on 12/21/23.
//

#ifndef NAV24_PARAMETER_HPP
#define NAV24_PARAMETER_HPP

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>

#include <opencv2/core/core.hpp>


namespace NAV24 {

    // Parameter class (ParamNode) is a doubly linked structure that reflects a settings file
    class Parameter;
    typedef std::shared_ptr<Parameter> ParamPtr;
    typedef std::weak_ptr<Parameter> ParamPtrW;
    class Parameter {
    public:
        //explicit Parameter(const std::string& name_);
        Parameter(const std::string& name_, const ParamPtr& parent_);

        // Insert child
        virtual void insertChild(const std::string& key, const ParamPtr& pChild);
        std::unordered_map<std::string, ParamPtrW> const& getAllChildren() { return children; }

        // Follows the key chain to find the parameter node, then returns the corresponding parameter node
        virtual ParamPtr read(const std::string& key);
        // Follows the key chain to find the parameter node, then resets it with paramPtr value
        virtual void write(const std::string& key, const ParamPtr& paramPtr);

        // Helper methods for key management
        static void splitKey(std::vector<std::string> &vKeys, const std::string& key, const std::string& delim = "/");
        static std::string& mergeKey(const std::vector<std::string>& vKey, const std::string& delim = "/");

        // Prints the node recursively from root to the last child
        virtual std::string printStr(const std::string& prefix = "") const;

        void setType(int type_) { type = type_; }
        int getType() { return type; }

        std::string const& getName() { return name; }
        void setName(const std::string& name_) { name = name_; }

    protected:
        ParamPtr getChild(const std::string& key);
        ParamPtr seekNode(const std::string& key);

        std::string name;
        int type;
        ParamPtrW parent;
        std::unordered_map<std::string, ParamPtrW> children;
    };

    template<typename T>
    class ParamType : public Parameter {
    public:
        ParamType<T>(const std::string& name_, const ParamPtr& parent_, const T& value) :
                Parameter(name_, parent_), mData(value) {}

        std::string printStr(const std::string &prefix = "") const override;

        void setValue(const T& data) { mData = data; }
        T const& getValue() { return mData; }

    private:
        T mData;
    };

    template<typename T>
    class ParamSeq : public Parameter {
    public:
        ParamSeq<T>(const std::string& name_, const ParamPtr& parent_, const std::vector<T>& value) :
                Parameter(name_, parent_), mvData(value) {}

        std::string printStr(const std::string &prefix = "") const override;

        std::vector<T> const& getValue() { return mvData; }
        void setValue(const std::vector<T>& vData) { mvData = vData; }

        void push_back(const T& val) { mvData.push_back(val); }
        T const& get(int idx) { return mvData[idx]; }

    private:
        std::vector<T> mvData;
    };
}

#endif //NAV24_PARAMETER_HPP
