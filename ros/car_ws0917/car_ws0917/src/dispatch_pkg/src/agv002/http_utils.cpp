#include "globals.h"

/* ====================================================== */
/* 工具函数                         */
/* ====================================================== */
/* ---------- 通用网络错误判断 ---------- */
inline bool isNetworkError(CURLcode code)
{
    switch (code)
    {
    case CURLE_COULDNT_RESOLVE_HOST:
    case CURLE_COULDNT_CONNECT:
    case CURLE_OPERATION_TIMEDOUT:
    case CURLE_COULDNT_RESOLVE_PROXY:
    case CURLE_SSL_CONNECT_ERROR:
        return true;
    default:
        return false;
    }
}

static size_t writeCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    size_t total = size * nmemb;
    static_cast<std::string *>(userp)->append(static_cast<char *>(contents), total);
    return total;
}

std::string nowStr()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

/* ====================================================== */
/* HTTP POST 请求函数                     */
/* ====================================================== */
bool httpPost(const std::string &url,
              const json &payload,
              std::string &resp_out,
              long &http_code_out,
              bool use_auth)
{
    for (int attempt = 0; attempt < 2; ++attempt)
    {
        CURL *curl = curl_easy_init();
        if (!curl)
            return false;

        struct curl_slist *headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        if (use_auth && !g_auth_token.empty())
        {
            headers = curl_slist_append(headers,
                                        ("Authorization: " + g_auth_token).c_str());
        }

        std::string payload_str = payload.dump();
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload_str.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, payload_str.size());
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resp_out);

        CURLcode res = curl_easy_perform(curl);
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code_out);
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);

        /* -------- 网络故障只打印，返回 false，不让程序退出 -------- */
        if (res != CURLE_OK)
        {
            if (isNetworkError(res))
            {
                ROS_WARN_STREAM("[httpPost] 网络异常("
                                << curl_easy_strerror(res)
                                << ")，稍后重试...请求地址"
                                << url.c_str());
            }
            else
            {
                ROS_ERROR_STREAM("[httpPost] curl 失败: "
                                 << curl_easy_strerror(res)
                                 << ")，稍后重试...请求地址"
                                 << url.c_str());
            }
            return false; // 关键：只返回 false，由上层决定是否忽略
        }

        // 如果是 token 无效（401 / 403），第一轮重试时刷新 token
        if (use_auth && (http_code_out == 401 || http_code_out == 403) && attempt == 0)
        {
            ROS_WARN("[httpPost] token 失效，尝试重新登录并重试...");
            if (!loginAndFetchToken())
            {
                ROS_ERROR("[httpPost] 重新登录失败");
                return false;
            }
            continue; // 重试第二次
        }

        break; // 成功或非 401/403，跳出重试
    }

    return true;
}

/**
 * @brief 更新AGV的当前位置点并立即上报状态
 * @param new_position 新的位置点代码字符串
 */
void updateAgvPosition(const std::string &new_position)
{
    // 1. 更新全局位置变量
    agvpostion = new_position;
    ROS_INFO_STREAM("AGV position updated to: " << agvpostion);

    // 2. 调用接口，将包含新位置的状态反馈给服务器
    postRebackAgvStatus();
}

/* ====================================================== */
/* 登录和获取token函数                     */
/* ====================================================== */
bool loginAndFetchToken()
{
    const std::string url = g_api_base_url + "/prod-api/user/apiLogin";
    json payload = {
        {"uuid", ""},
        {"username", g_api_username},
        {"password", g_api_password},
        {"code", ""}};

    std::string resp;
    long httpCode = 0;

    // 登录接口本身不需要带 Authorization 头，所以 use_auth = false
    if (!httpPost(url, payload, resp, httpCode, /*use_auth=*/false) || httpCode != 200)
    {
        ROS_ERROR_STREAM("[login] 请求失败，HTTP " << httpCode);
        return false;
    }

    try
    {
        auto j = json::parse(resp);
        // 这里根据后端实际返回格式取 token
        if (j.contains("token"))
        {
            g_auth_token = j["token"].get<std::string>();
        }
        else if (j.contains("data") && j["data"].contains("token"))
        {
            g_auth_token = j["data"]["token"].get<std::string>();
        }
        else
        {
            ROS_ERROR("[login] JSON 中未找到 token 字段");
            return false;
        }
        ROS_INFO_STREAM("[login] 获取 token 成功");
        return true;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("[login] 解析 JSON 失败: " << e.what());
        return false;
    }
}
