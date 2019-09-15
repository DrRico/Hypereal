#include "jsonobject.h"

#include <cstring>
#include <cstdio>

//#include "bug.h"

bool JsonObject::FromJson(const Json::Value &) {
    return false;
}

Json::Value JsonObject::ToJson() const {
    return Json::Value();
}

bool JsonObject::FromString(const char *s) {
    Json::Reader r;
    Json::Value v;
	if (!r.parse(s, s + std::strlen(s), v, false)) {
#ifdef _DEBUG
		auto x = r.getStructuredErrors();
		//bug(std::string("Json Parser Failed:\n------------------------------\n") + /* bugAddLineNumber(s) +*/
		//	"\n------------------------------\n" + r.getFormattedErrorMessages());
#endif
        return false;
	}
    return FromJson(v);
}

std::string JsonObject::ToString() const {
    return ToJson().toStyledString();
}

bool JsonObject::Save(const char *dir) {
    FILE *f = std::fopen(dir, "w");
    if (f == nullptr) {
        return false;
    }
    auto s = ToString();
    std::fwrite(s.data(), sizeof(char), s.length(), f);
    std::fclose(f);
    return true;
}

bool JsonObject::Load(const char *dir) {
    FILE *f = std::fopen(dir, "r");
    if (f == nullptr) {
        return false;
    }
    
    std::fseek(f, 0, SEEK_END);
    size_t fsize = std::ftell(f);
    std::rewind(f);
    auto buffer = new char [fsize + 1];
    auto nread = std::fread(buffer, 1, fsize, f);
    if (nread == 0) {
        delete [] buffer;
        return false;
    }
    if (!FromString(buffer)) {
        delete [] buffer;
        return false;
    }
    delete [] buffer;
    std::fclose(f);
    return true;
}

