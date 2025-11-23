module LASER (
    input CLK,
    input RST,
    input [3:0] X,
    input [3:0] Y,
    output reg [3:0] C1X,
    output reg [3:0] C1Y,
    output reg [3:0] C2X,
    output reg [3:0] C2Y,
    output reg DONE
);

    //================================================================
    // 1. 參數與狀態定義
    //================================================================
    localparam S_IDLE           = 4'd0;
    localparam S_READ           = 4'd1;
    localparam S_INIT_C1        = 4'd2;
    localparam S_CALC_C1        = 4'd3;
    localparam S_UPDATE_C1      = 4'd4;
    localparam S_INIT_C2        = 4'd5;
    localparam S_CALC_C2        = 4'd6;
    localparam S_UPDATE_C2      = 4'd7;
    localparam S_CHECK_CONV     = 4'd8;
    localparam S_DONE           = 4'd9;
    localparam S_WAIT           = 4'd10;

    //================================================================
    // 2. 變數宣告
    //================================================================
    reg [3:0] state;
    reg [5:0] pixel_count;
    
    // 記憶體
    reg [3:0] X_MEM [0:39];
    reg [3:0] Y_MEM [0:39];

    // 搜尋變數
    reg [5:0] mark_data;        
    reg [3:0] index_X, index_Y; 
    
    // 計分變數
    reg [5:0] current_hit_count;
    reg [5:0] best_hit_count;
    
    // 最佳解與收斂檢查
    reg [3:0] best_c1x, best_c1y;
    reg [3:0] best_c2x, best_c2y;
    reg [3:0] old_c1x, old_c1y, old_c2x, old_c2y;

    // 組合邏輯訊號 (Wire)
    reg p0_scan, p1_scan, p2_scan, p3_scan;   // 點是否在「掃描圓」內
    reg p0_fix,  p1_fix,  p2_fix,  p3_fix;    // 點是否在「固定圓」內
    reg hit0, hit1, hit2, hit3;               // 最終命中 (OR 運算結果)
    
    // 用來切換「固定圓」是誰
    reg [3:0] fixed_cx, fixed_cy;

    // 初始化 (防止 Time 0 崩潰)
    initial begin
        state = S_IDLE;
        pixel_count = 0;
        mark_data = 0;
        index_X = 0; index_Y = 0;
        current_hit_count = 0; best_hit_count = 0;
        best_c1x = 0; best_c1y = 0; best_c2x = 0; best_c2y = 0;
    end

    //================================================================
    // 3. 單層 FSM (控制路徑)
    //================================================================
    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            state <= S_IDLE;
            pixel_count <= 0;
            DONE <= 0;
            C1X <= 0; C1Y <= 0; C2X <= 0; C2Y <= 0;
            
            mark_data <= 0;
            index_X <= 0; index_Y <= 0;
            current_hit_count <= 0; best_hit_count <= 0;
            best_c1x <= 0; best_c1y <= 0; best_c2x <= 0; best_c2y <= 0;
        end
        else begin
            case(state)
                S_IDLE: begin
                    DONE <= 0;
                    state <= S_READ;
                    // 清空最佳解
                    best_c1x <= 0; best_c1y <= 0; best_c2x <= 0; best_c2y <= 0;
                    X_MEM[pixel_count] <= X;
                    Y_MEM[pixel_count] <= Y;
                    pixel_count <= pixel_count + 1;
                end
                
                S_READ: begin
                    X_MEM[pixel_count] <= X;
                    Y_MEM[pixel_count] <= Y;
                    if (pixel_count >= 39) begin
                        pixel_count <= 0;
                        state <= S_INIT_C1; 
                    end
                    else begin
                        pixel_count <= pixel_count + 1;
                    end
                end

                // --- C1 搜尋初始化 ---
                S_INIT_C1: begin
                    index_X <= 0;
                    index_Y <= 0;
                    best_hit_count <= 0; 
                    
                    // 記錄舊值查收斂
                    old_c1x <= best_c1x; old_c1y <= best_c1y;
                    old_c2x <= best_c2x; old_c2y <= best_c2y;

                    state <= S_CALC_C1;
                    mark_data <= 0;
                    current_hit_count <= 0;
                end

                // --- C1 計算 (核心改變：不查表，直接用 hit0~3 加總) ---
                S_CALC_C1: begin
                    current_hit_count <= current_hit_count + hit0 + hit1 + hit2 + hit3;
                    
                    if (mark_data == 36) begin
                        state <= S_UPDATE_C1;
                        mark_data <= 0;
                    end
                    else begin
                        mark_data <= mark_data + 4;
                    end
                end

                // --- C1 更新 ---
                S_UPDATE_C1: begin
                    if (current_hit_count >= best_hit_count) begin
                        best_hit_count <= current_hit_count;
                        best_c1x <= index_X;
                        best_c1y <= index_Y;
                    end
                    
                    current_hit_count <= 0;

                    if (index_X == 15 && index_Y == 15) begin
                        state <= S_INIT_C2;
                    end
                    else begin
                        if (index_X == 15) begin
                            index_X <= 0;
                            index_Y <= index_Y + 1;
                        end else begin
                            index_X <= index_X + 1;
                        end
                        state <= S_CALC_C1; 
                    end
                end

                // --- C2 搜尋初始化 ---
                S_INIT_C2: begin
                    index_X <= 0;
                    index_Y <= 0;
                    best_hit_count <= 0; 
                    state <= S_CALC_C2;
                    mark_data <= 0;
                    current_hit_count <= 0;
                end

                // --- C2 計算 ---
                S_CALC_C2: begin
                    current_hit_count <= current_hit_count + hit0 + hit1 + hit2 + hit3;
                    
                    if (mark_data == 36) begin
                        state <= S_UPDATE_C2;
                        mark_data <= 0;
                    end
                    else begin
                        mark_data <= mark_data + 4;
                    end
                end

                // --- C2 更新 ---
                S_UPDATE_C2: begin
                    if (current_hit_count >= best_hit_count) begin
                        best_hit_count <= current_hit_count;
                        best_c2x <= index_X;
                        best_c2y <= index_Y;
                    end

                    current_hit_count <= 0;

                    if (index_X == 15 && index_Y == 15) begin
                        state <= S_CHECK_CONV; 
                    end
                    else begin
                        if (index_X == 15) begin
                            index_X <= 0;
                            index_Y <= index_Y + 1;
                        end else begin
                            index_X <= index_X + 1;
                        end
                        state <= S_CALC_C2;
                    end
                end

                // --- 收斂檢查 ---
                S_CHECK_CONV: begin
                    if (old_c1x == best_c1x && old_c1y == best_c1y && 
                        old_c2x == best_c2x && old_c2y == best_c2y) begin
                        state <= S_DONE;
                    end
                    else begin
                        state <= S_INIT_C1;
                    end
                end

                S_DONE: begin
                    C1X <= best_c1x; C1Y <= best_c1y;
                    C2X <= best_c2x; C2Y <= best_c2y;
                    DONE <= 1;
                    state <= S_WAIT;
                end
                
                S_WAIT: begin
                    DONE <= 0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    //================================================================
    // 4. 運算引擎 (Data Path - 這裡改用 Plan B 的重算策略)
    //================================================================
    
    // 曼哈頓距離檢查函式 (你喜歡的算法)
    function check_hit;
        input [3:0] tx, ty, cx, cy;
        reg [3:0] abs_dx, abs_dy;
        begin
            abs_dx = (tx >= cx) ? (tx - cx) : (cx - tx);
            abs_dy = (ty >= cy) ? (ty - cy) : (cy - ty);
            if ( (abs_dx + abs_dy <= 4) || (abs_dx == 2 && abs_dy == 3) || (abs_dx == 3 && abs_dy == 2) )
                check_hit = 1'b1;
            else
                check_hit = 1'b0;
        end
    endfunction

    // 決定誰是 "固定圓"
    always @(*) begin
        if (state == S_CALC_C1) begin
            // 找 C1 時，C2 是固定的
            fixed_cx = best_c2x;
            fixed_cy = best_c2y;
        end
        else begin 
            // 找 C2 時，C1 是固定的 (預設)
            fixed_cx = best_c1x;
            fixed_cy = best_c1y;
        end
    end

    // 平行計算邏輯 (移除 MUX，改用兩組 check_hit)
    always @(*) begin
        // 1. 檢查是否在「掃描圓 (Current)」內
        p0_scan = check_hit(X_MEM[mark_data],   Y_MEM[mark_data],   index_X, index_Y);
        p1_scan = check_hit(X_MEM[mark_data+1], Y_MEM[mark_data+1], index_X, index_Y);
        p2_scan = check_hit(X_MEM[mark_data+2], Y_MEM[mark_data+2], index_X, index_Y);
        p3_scan = check_hit(X_MEM[mark_data+3], Y_MEM[mark_data+3], index_X, index_Y);

        // 2. 檢查是否在「固定圓 (Fixed)」內 --> 這就是 Plan B 的精髓 (重算取代查表)
        p0_fix = check_hit(X_MEM[mark_data],   Y_MEM[mark_data],   fixed_cx, fixed_cy);
        p1_fix = check_hit(X_MEM[mark_data+1], Y_MEM[mark_data+1], fixed_cx, fixed_cy);
        p2_fix = check_hit(X_MEM[mark_data+2], Y_MEM[mark_data+2], fixed_cx, fixed_cy);
        p3_fix = check_hit(X_MEM[mark_data+3], Y_MEM[mark_data+3], fixed_cx, fixed_cy);

        // 3. 合併結果 (在掃描圓 OR 在固定圓 = 有被覆蓋)
        hit0 = p0_scan || p0_fix;
        hit1 = p1_scan || p1_fix;
        hit2 = p2_scan || p2_fix;
        hit3 = p3_scan || p3_fix;
    end

endmodule