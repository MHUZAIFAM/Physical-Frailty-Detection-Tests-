void speak(const std::string& text) {
    std::thread([text]() {
        ISpVoice* pVoice = NULL;

        if (FAILED(::CoInitialize(NULL))) {
            std::cerr << "Failed to initialize COM library." << std::endl;
            return;
        }

        HRESULT hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void**)&pVoice);
        if (SUCCEEDED(hr)) {
            wchar_t wtext[1024];
            size_t convertedChars = 0;
            mbstowcs_s(&convertedChars, wtext, sizeof(wtext) / sizeof(wchar_t), text.c_str(), _TRUNCATE);

            // Use SPF_ASYNC for non-blocking speech
            pVoice->Speak(wtext, SPF_ASYNC, NULL);

            // Wait for speech to complete (optional, remove if unnecessary)
            pVoice->WaitUntilDone(INFINITE);

            pVoice->Release();
            pVoice = NULL;
        } else {
            std::cerr << "Failed to create voice instance." << std::endl;
        }
        CoUninitialize();
    }).detach(); // Detach the thread so it runs independently
}
