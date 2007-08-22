#include "stdinc.h"

//#ifdef HAVE_CURL
static size_t CPLHTTPWriteFunc(void *buffer, size_t count, size_t nmemb, void *req) {
	CPLHTTPRequest *psRequest = reinterpret_cast<CPLHTTPRequest *>(req);
	size_t size = count * nmemb;

	if (size == 0) return 0;

	const size_t required_size = psRequest->nDataLen + size + 1;
	if (required_size > psRequest->nDataAlloc) {
		size_t new_size = required_size * 2;
		if (new_size < 512) new_size = 512;
		psRequest->nDataAlloc = new_size;
		psRequest->pabyData = reinterpret_cast<GByte *>(VSIRealloc(psRequest->pabyData, new_size));
		if (psRequest->pabyData == NULL) {
			psRequest->pszError = CPLStrdup(CPLString().Printf("Out of memory allocating %u bytes for HTTP data buffer.", static_cast<int>(new_size)));
			psRequest->nDataAlloc = 0;
			psRequest->nDataLen = 0;
			return 0;
		}
	}
	memcpy(psRequest->pabyData + psRequest->nDataLen, buffer, size);
	psRequest->nDataLen += size;
	psRequest->pabyData[psRequest->nDataLen] = 0;
	return nmemb;
}
//#endif /* def HAVE_CURL */

void CPLHTTPInitializeRequest(CPLHTTPRequest *psRequest, const char *pszURL, const char **papszOptions) {
	psRequest->pszURL = CPLStrdup(pszURL);
	psRequest->papszOptions = CSLDuplicate(const_cast<char **>(papszOptions));
	psRequest->nStatus = 0;
	psRequest->pszContentType = 0;
	psRequest->pszError = 0;
	psRequest->pabyData = 0;
	psRequest->nDataLen = 0;
	psRequest->nDataAlloc = 0;
	psRequest->m_curl_handle = 0;
	psRequest->m_headers = 0;
	psRequest->m_curl_error = 0;

	psRequest->m_curl_handle = curl_easy_init();
	if (psRequest->m_curl_handle == NULL) {
		CPLError(CE_Fatal, CPLE_AppDefined, "CPLHTTPInitializeRequest(): Unable to create CURL handle.\n");
	}

	curl_easy_setopt(psRequest->m_curl_handle, CURLOPT_URL, psRequest->pszURL);

	const char *timeout = CSLFetchNameValue(const_cast<char **>(psRequest->papszOptions), "TIMEOUT");
	if (timeout != NULL) {
		curl_easy_setopt(psRequest->m_curl_handle, CURLOPT_TIMEOUT, atoi(timeout));
	}

	const char *headers = CSLFetchNameValue(const_cast<char **>(psRequest->papszOptions), "HEADERS");
	if (headers != NULL) {
		psRequest->m_headers = curl_slist_append(psRequest->m_headers, headers);
		curl_easy_setopt(psRequest->m_curl_handle, CURLOPT_HTTPHEADER, psRequest->m_headers);
	}

	/* Enable following redirections.  Requires libcurl 7.10.1 at least */
	curl_easy_setopt(psRequest->m_curl_handle, CURLOPT_FOLLOWLOCATION, 1);
	curl_easy_setopt(psRequest->m_curl_handle, CURLOPT_MAXREDIRS, 10);

	/* NOSIGNAL should be set to true for timeout to work in multithread
	environments on Unix, requires libcurl 7.10 or more recent.
	(this force avoiding the use of sgnal handlers) */
#ifdef CURLOPT_NOSIGNAL
	curl_easy_setopt(psRequest->m_curl_handle, CURLOPT_NOSIGNAL, 1);
#endif

	curl_easy_setopt(psRequest->m_curl_handle, CURLOPT_WRITEDATA, psRequest);
	curl_easy_setopt(psRequest->m_curl_handle, CURLOPT_WRITEFUNCTION, CPLHTTPWriteFunc);

	psRequest->m_curl_error = reinterpret_cast<char *>(CPLMalloc(CURL_ERROR_SIZE + 1));
	curl_easy_setopt(psRequest->m_curl_handle, CURLOPT_ERRORBUFFER, psRequest->m_curl_error);
}

void CPLHTTPCleanupRequest(CPLHTTPRequest *psRequest) {
	if (psRequest->m_curl_handle) {
		curl_easy_cleanup(psRequest->m_curl_handle);
		psRequest->m_curl_handle = 0;
	}
	if (psRequest->m_headers) {
		curl_slist_free_all(psRequest->m_headers);
		psRequest->m_headers = 0;
	}
	if (psRequest->m_curl_error) {
		CPLFree(psRequest->m_curl_error);
		psRequest->m_curl_error = 0;
	}

	if (psRequest->pszContentType) {
		CPLFree(psRequest->pszContentType);
		psRequest->pszContentType = 0;
	}
	if (psRequest->pszError) {
		CPLFree(psRequest->pszError);
		psRequest->pszError = 0;
	}
	if (psRequest->pabyData) {
		CPLFree(psRequest->pabyData);
		psRequest->pabyData = 0;
		psRequest->nDataLen = 0;
		psRequest->nDataAlloc = 0;
	}
	if (psRequest->papszOptions) {
		CSLDestroy(psRequest->papszOptions);
		psRequest->papszOptions = 0;
	}
	if (psRequest->pszURL) {
		CPLFree(psRequest->pszURL);
		psRequest->pszURL = 0;
	}
}

void CPLHTTPFetchMulti(CPLHTTPRequest *pasRequest, int nRequestCount) {
	CURLM *curl_multi = 0;
	int still_running;

	curl_multi = curl_multi_init();
	if (curl_multi == NULL) {
		CPLError(CE_Fatal, CPLE_AppDefined, "CPLHTTPFetchMulti(): Unable to create CURL multi-handle.\n");
	}

	for (int i = 0; i < nRequestCount; ++i) {
		CPLHTTPRequest *const psRequest = &pasRequest[i];

		curl_multi_add_handle(curl_multi, psRequest->m_curl_handle);
	}

	while (curl_multi_perform(curl_multi, &still_running) == CURLM_CALL_MULTI_PERFORM);
	while (still_running) {
		struct timeval timeout;
		fd_set fdread, fdwrite, fdexcep;
		int maxfd;

		FD_ZERO(&fdread);
		FD_ZERO(&fdwrite);
		FD_ZERO(&fdexcep);
		curl_multi_fdset(curl_multi, &fdread, &fdwrite, &fdexcep, &maxfd);
		timeout.tv_sec = 0;
		timeout.tv_usec = 100000;
		select(maxfd + 1, &fdread, &fdwrite, &fdexcep, &timeout);
		while (curl_multi_perform(curl_multi, &still_running) == CURLM_CALL_MULTI_PERFORM);
	}

	for (int i = 0; i < nRequestCount; ++i) {
		CPLHTTPRequest *const psRequest = &pasRequest[i];

		long response_code = 0;
		curl_easy_getinfo(psRequest->m_curl_handle, CURLINFO_RESPONSE_CODE, &response_code);
		psRequest->nStatus = response_code;

		char *content_type = 0;
		curl_easy_getinfo(psRequest->m_curl_handle, CURLINFO_CONTENT_TYPE, &content_type);
		if (content_type) psRequest->pszContentType = CPLStrdup(content_type);

		if ((psRequest->pszError == NULL) && (psRequest->m_curl_error != NULL) && (psRequest->m_curl_error[0] != '\0')) {
			psRequest->pszError = CPLStrdup(psRequest->m_curl_error);
		}

		curl_multi_remove_handle(curl_multi, pasRequest[i].m_curl_handle);
	}
	curl_multi_cleanup(curl_multi);
}
