#include "sensors.h"
#include <cmath>

#define CONSTELLATION_N_POINTS 200

Constellation::Constellation(int sat, int N_points):
    N_points(N_points),
    sbuff(N_points)
{
    plot_name = std::string("Sat ") + std::to_string(sat+1);
}

Constellation::~Constellation()
{
}

void Constellation::tab_item(void)
{
    if(ImGui::BeginTabItem(plot_name.c_str()))
    {
        if(ImPlot::BeginPlot("##Plot", ImVec2(400,400),
                              ImPlotFlags_Equal | ImPlotFlags_CanvasOnly | ImPlotFlags_NoFrame | ImPlotFlags_NoInputs)){
            ImVector<ImVec2> normalized;
            normalized.push_back(ImVec2(0.0,0.0));
            if(sbuff.Data.size()){
                float abs_max = 0.0f;
                for(int i=0; i<sbuff.Data.size();i++){
                    float x2 = sbuff.Data[i].x;
                    x2*=x2;
                    float y2 = sbuff.Data[i].y;
                    y2*=y2;
                    float abs_1 = sqrtf(x2 + y2);
                    if(abs_1>abs_max){
                        abs_max = abs_1;
                    }
                }
                abs_max *= 1.1f;
                normalized = sbuff.Data;
                for(int i=0;i<normalized.size();i++){
                    normalized[i].x /= abs_max;
                    normalized[i].y /= abs_max;
                }
            }
            ImPlotAxisFlags aflags = ImPlotAxisFlags_NoDecorations | ImPlotAxisFlags_NoMenus
                                     | ImPlotAxisFlags_NoInitialFit;
            ImPlot::SetupAxis(ImAxis_X1, "Real", aflags);
            ImPlot::SetupAxis(ImAxis_Y1, "Imag", aflags);
            ImPlot::SetupAxisLimits(ImAxis_X1, -1.0, 1.0, ImPlotCond_Once);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -1.0, 1.0, ImPlotCond_Once);
            ImPlot::PlotScatter(plot_name.c_str(), &normalized[0].x, &normalized[0].y,
                                normalized.size(),0, 0, sizeof(ImVec2));

            ImPlot::EndPlot();
        }
        ImGui::EndTabItem();
    }
}

void Constellation::data_point(std::complex<float> x)
{
    sbuff.AddPoint(x.real(), x.imag());
}

Sensors::Sensors(void)
{
    for(int s=0;s<N_SATELLITES;s++){
        sat_slot[s] = -1;
    }

    thread_enabled = true;
    thread = std::thread(&Sensors::thread_func, this);
}

Sensors::~Sensors(void)
{
    thread_enabled = false;
    thread.join();
}

// Callback to handle GLFW errors
void glfw_error_callback(int error, const char* description) { std::cerr << "GLFW Error " << error << ": " << description << std::endl; }

void Sensors::thread_func(void)
{
    // Setup error callback
    glfwSetErrorCallback(glfw_error_callback);

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return;
    }

    // Setup OpenGL version
#if defined(__APPLE__)
    // GL 3.2 + GLSL 150 (MacOS)
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);           // Required on MacOS
#else
    // GL 3.0 + GLSL 130 (Windows and Linux)
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

    // Create window
    GLFWwindow* window = glfwCreateWindow(1200, 800, "GPS Sensors", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0); // Disable vsync

    // Setup context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    // Setup style
    ImGui::StyleColorsDark();

    // Setup backend
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Main loop
    while (thread_enabled && !glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // update the state of the satellites
        sats_update();

        // Start frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Plot the satellite sensors
        int w, h;
        glfwGetWindowSize(window, &w, &h);
        ImVec2 window_size(w, h);

        ImGui::SetNextWindowSize(window_size);
        ImGui::SetNextWindowPos({0,0});

        ImGui::Begin("GPS",nullptr,ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize);
        ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
        if (ImGui::BeginTabBar("SatTabBar", tab_bar_flags))
        {
            sat_tab_items();
            ImGui::EndTabBar();
        }

        ImGui::End();

        // Render
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Swap buffers
        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
}

void Sensors::sat_tab_items(void)
{
    for(int s=0;s<N_SATELLITES;s++){
        if(slots[s]){
            slots[s]->constellation.tab_item();
        }
    }
}

void Sensors::sats_update(void)
{
    int N_msgs = queue.size();
    for(int i=0;i<N_msgs;i++){
        std::unique_ptr<SensorMsg> msg = queue.pop();
        switch(msg->type){
        case SMT_ADD:
            add_sat(static_cast<SensorMsgAdd*>(msg.get()));
            break;
        case SMT_DEL:
            del_sat(static_cast<SensorMsgDel*>(msg.get()));
            break;
        case SMT_DATA:
            sat_data(static_cast<SensorMsgData*>(msg.get()));
            break;
        }
    }
}

void Sensors::add_sat(SensorMsgAdd *msg)
{
    // test for already allocated
    if(sat_slot[msg->sat]>=0)
        return;

    // find the next available slot
    int slot;
    for(slot=0;slot<N_SATELLITES;slot++){
        if(!slots[slot])
            break;
    }

    // allocate the sensors for the slot
    slots[slot].reset(new SensorSlot(msg->sat, CONSTELLATION_N_POINTS));
    sat_slot[msg->sat] = slot;
}

void Sensors::del_sat(SensorMsgDel *msg)
{
    // check for already deallocated
    if(sat_slot[msg->sat]<0)
        return;

    int slot = sat_slot[msg->sat];
    // deallocate this slot
    slots[slot].reset(nullptr);
    // indicate that this slot is available
    sat_slot[slot] = -1;
}

void Sensors::sat_data(SensorMsgData *msg)
{
    // check for a valid slot
    int slot = sat_slot[msg->sat];
    if(slot<0)
        return;

    for(int i=0;i<msg->N_data;i++){
        slots[slot]->constellation.data_point(msg->iq[i]);
    }
}

void Sensors::send_add_sat(int sat)
{
    queue.push(std::make_unique<SensorMsgAdd>(sat));
}

void Sensors::send_del_sat(int sat)
{
    queue.push(std::make_unique<SensorMsgDel>(sat));
}

void Sensors::send_sat_data(int sat, int N_data, std::unique_ptr<std::complex<float>[]> iq)
{
    queue.push(std::make_unique<SensorMsgData>(sat, N_data, std::move(iq)));
}

