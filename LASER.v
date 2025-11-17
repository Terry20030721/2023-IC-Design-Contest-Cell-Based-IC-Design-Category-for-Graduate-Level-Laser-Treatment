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

//宣告state的狀態並賦予值
localparam S_IDLE = 2'b00; //Idle狀態
localparam S_READ = 2'b01; //Read狀態
localparam S_CALC = 2'b10; //Calculate狀態 這邊還有內圈的state之後會去設置
localparam S_DONE = 2'b11; //Done狀態

reg [1:0] state; //紀錄目前state

reg [5:0] pixel_count; //計算目前掃描到第幾個pixel 共40個所以其實是配合底下MEM從0~39所以需要6bits

//宣告一個記憶體，儲存40個標記點的位置
reg [3:0] X_MEM[0:39]; //儲存座標的X值，櫃子大小為4bits可以表達16x16的範圍，其中XMEM有40個櫃子
reg [3:0] Y_MEM[0:39]; //儲存座標的Y值，櫃子大小為4bits可以表達16x16的範圍，其中YMEM有40個櫃子


reg calc_done; //為何需要這個？因為S_CALC是外圈(老闆)的狀態，calc_done是代表圈(員工)的狀態，當員工完成工作後要告訴老闆我完成了


always @(posedge CLK) begin
    if (RST) begin //收到初始化的訊號
        state <= S_IDLE; //狀態回到Idle 準備開始進入下一階段
        DONE <= 1'b0; //DONE訊號拉低
        pixel_count <=0; //pixel_count歸零 -> input點的數量歸0
        calc_done <= 1'b0; //計算完成訊號拉低
    end 
    else begin //如果沒有收到初始化訊號就要來檢查state做出對應的工作
        case (state)

            S_IDLE: begin //Idle狀態
                DONE <= 1'b0; //DONE訊號拉低
                pixel_count <= 0; //pixel_count歸零 -> input點的數量歸0
                if (!RST) begin //如果沒有收到初始化訊號就進入Read狀態
                    state <= S_READ;
                end
            end

            S_READ: begin //Read狀態
                //將輸入的X,Y座標存入記憶體中
                // 在testbench中每個clock cycle會提供一組X,Y座標
                X_MEM[pixel_count] <= X; //將目前的X值存入X_MEM
                Y_MEM[pixel_count] <= Y; //將目前的Y值存入Y_MEM
                pixel_count <= pixel_count + 1; //掃描到的點數量+1

                if (pixel_count == 6'd39) begin //如果已經掃描到第40個點，進入Calculate狀態
                    state <= S_CALC;
                end
            end

            S_CALC: begin //Calculate狀態

                if (calc_done) begin // 透過calc_done了解計算狀態，如果計算完成，進入Done狀態
                    state <= S_DONE;
                    DONE <=1'b1;
                end

                else begin
                    state <= S_CALC; //如果還沒計算完成，繼續待在Calculate狀態
                    //這邊會因為條件符合，繼續進行計算
                end
            end

            S_DONE: begin //Done狀態
                DONE <= 0; //DONE訊號拉回因為根據testbench的需求DONE只會維持一個clock cycle
                state <= S_IDLE; //回到Idle狀態準備下一輪的運算
            end

            default: begin
                state <= S_IDLE; //預設回到Idle狀態
            end

        endcase
    end
end

//======= Algorithm Engine ========


//計算狀態的宣告
localparam calc_idle = 4'b0000;
localparam calc_set_c1 = 4'b0001;
localparam calc_c1_hit_count = 4'b0010;
localparam calc_c1_compare = 4'b0011;
localparam calc_set_c2 = 4'b100;
localparam calc_c2_hit_count = 4'b0101;
localparam calc_c2_compare = 4'b0110;
localparam calc_check_convergence = 4'b0111;
localparam calc_finish = 4'b1000;

//宣告計算狀態的變數
reg [3:0] calc_state; //狀態表達用4個bits



//儲存距離計算的變數
reg [4:0] X_dis; //這邊會需要５bits因為有正負問題
reg [4:0] Y_dis; //這邊會需要５bits因為有正負問題
reg [3:0] X_abs; //所以這邊是4bis因為最大值是16
reg [3:0] Y_abs; //所以這邊是4bis因為最大值是16

//目前要計算的座標位置
reg [3:0] index_X, index_Y; // 遍歷所有圓心位置的變數
reg [5:0] mark_data; //用來標記目前正在計算第幾個記憶體位置的變數
reg [5:0] current_hit_count; //紀錄c1命中數量的變數
reg [5:0] best_hit_count; //紀錄目前最佳命中數量的變數 在做c1 & c2 的 compare會用到，紀錄這次迭代時c1 or c2的最佳命中數量
reg [3:0] c1_best_x, c1_best_y; //紀錄目前最佳C1座標的變數
reg [3:0] c2_best_x, c2_best_y; //紀錄目前最佳C2座標的變數 

//這邊比較特別，這邊是在儲存舊的best C1 & C2 因為可以用來檢查是否已達收斂！！
reg [3:0] old_c1_best_x, old_c1_best_y;
reg [3:0] old_c2_best_x, old_c2_best_y;

// --- 使用「標記法」記憶體，分別記錄哪些點被c1 or c2 所涵蓋---
reg [0:39] max_shade1; // C1 的標記
reg [0:39] max_shade2; // C2 的標記
reg [0:39] temp_shade; // 暫時紀錄當前c1 or c2 的標記

reg hit; //紀錄是否命中的變數


always @(posedge CLK) begin
    if (RST) begin
        //  這邊要注意， 所有參數都初始化，避免模擬時出現卡死的情況
        index_X <= 4'b0000;
        index_Y <= 4'b0000;
        mark_data <= 6'b000000;
        current_hit_count <= 6'b000000;
        best_hit_count <= 6'b000000;
        c1_best_x <= 4'b0000;
        c1_best_y <= 4'b0000;
        c2_best_x <= 4'b0000;
        c2_best_y <= 4'b0000;
        old_c1_best_x <= 4'b0000;
        old_c1_best_y <= 4'b0000;
        old_c2_best_x <= 4'b0000;
        old_c2_best_y <= 4'b0000;
        max_shade1 <= 40'b0;
        max_shade2 <= 40'b0;
        temp_shade <= 40'b0;
        C1X <= 4'b0000;
        C1Y <= 4'b0000;
        C2X <= 4'b0000;
        C2Y <= 4'b0000;
        calc_state <= calc_idle;
    end
    else if (state == S_CALC) begin
        //這邊放置計算C1X,C1Y,C2X,C2Y的演算法
        //需要演算法透過FSM的方式來進行，所以依然需要計算狀態的設置
        case (calc_state)
            calc_idle: begin
                //初始化計算狀態
                calc_state <= calc_set_c1;
            end

            calc_set_c1: begin
                //設定C1的初始值

                //把上一輪的C1,C2值存起來以便檢查收斂
                old_c1_best_x <= c1_best_x;
                old_c1_best_y <= c1_best_y;
                old_c2_best_x <= c2_best_x;
                old_c2_best_y <= c2_best_y;
                
                //把所有數值重置
                //重置點
                index_X <= 0;
                index_Y <= 0;

                //重置要被計算是否命中的標記
                mark_data <= 0;

                //重置命中的相關參數
                current_hit_count <= 0;
                best_hit_count <= 0;
                temp_shade <= 0;

                //進入計算C1命中數量的狀態並開始下次的迭代
                calc_state <= calc_c1_hit_count;
            end

            calc_c1_hit_count: begin
                //計算C1的命中數量
                //規則: 檢查 (C1_test || C2_best)
                //max_shade2 是上次迭代得到的結果
                if (hit==1 || max_shade2[mark_data]==1) begin //只要命中或是被shade2涵蓋到就算命中
                    current_hit_count <= current_hit_count + 1;
                    temp_shade[mark_data] <= hit; //根據此點是否實際被命中做決定
                end
                else begin
                    temp_shade[mark_data] <= 1'b0; //未命中
                end
                // 內層迴圈 (40 個標的物)
                if (mark_data == 39) begin
                    calc_state <= calc_c1_compare; // 40 個點算完了，去比較
                    mark_data  <= 0;
                end
                else begin
                    calc_state <= calc_c1_hit_count; // 還沒，繼續
                    mark_data  <= mark_data + 1;
                end
            end

            calc_c1_compare: begin
                //比較C1的命中數量，更新C1值
                // 比較並儲存 C1
                if (current_hit_count > best_hit_count) begin
                    best_hit_count <= current_hit_count;
                    c1_best_x  <= index_X;
                    c1_best_y  <= index_Y;
                    max_shade1 <= temp_shade; // **儲存 C1 標記**
                end
                temp_shade <= 0; // 重置暫存標記
                current_hit_count <= 0; // 重置 hit

                // 外層迴圈 (256 個圓心)
                if (index_X == 15 && index_Y == 15) begin
                    calc_state <= calc_set_c2; // C1 找完了，去 C2
                    //跳轉到下一階段來重置
                end
                else begin
                    // ... (index_X, Y 遞增) ...
                    if (index_X == 15) begin
                        index_X <= 0;
                        index_Y <= index_Y + 1;
                    end
                    else begin
                        index_X <= index_X + 1;
                    end
                    calc_state <= calc_c1_hit_count; // 回去 Hit Count
                end
            end

            calc_set_c2: begin
                //設定C2的初始值
                //把所有數值重置
                index_X <= 0;
                index_Y <= 0;
                mark_data <= 0;
                current_hit_count <= 0;
                best_hit_count <= 0;
                temp_shade <= 0;
                //進入計算C2命中數量的狀態並開始下次的迭代
                calc_state <= calc_c2_hit_count;
            end

            calc_c2_hit_count: begin
                //計算C2的命中數量
                if (hit == 1 | max_shade1[mark_data] == 1) begin
                    current_hit_count <= current_hit_count + 1;
                    temp_shade[mark_data] <= hit;
                end
                else begin
                    temp_shade[mark_data] <= 1'b0;
                end
                // 內層迴圈 (40 個標的物)
                if (mark_data == 39) begin 
                    calc_state <= calc_c2_compare; //40個點是否韓蓋都計算完了就去比較
                    mark_data <= 0; //重置mark_data以便下次計算
                end
                else begin
                    calc_state <= calc_c2_hit_count; //還沒，繼續
                    mark_data <= mark_data + 1; //這邊是mark_data的遞增
                end

            end

            calc_c2_compare: begin
                //比較C2的命中數量，更新C2值
                //並儲存 C2
                if (current_hit_count > best_hit_count) begin
                    best_hit_count <= current_hit_count;
                    c2_best_x <= index_X;
                    c2_best_y <= index_Y;
                    max_shade2 <= temp_shade; // **儲存 C2 標記**
                end
                temp_shade <= 0; //重置temp_shade以便下次紀錄temp_shade
                current_hit_count <= 0; //重置current_hit_count以便下次計算

                if (index_X == 15 && index_Y == 15) begin
                    // C2 找完了，結束計算
                    calc_state <= calc_check_convergence; //去檢查收斂
                end
                else begin
                    // (index_X, Y 遞增)
                    if (index_X == 15) begin
                        index_X <= 0;
                        index_Y <= index_Y + 1;
                    end
                    else begin
                        index_X <= index_X + 1;
                    end
                calc_state <= calc_c2_hit_count; //回去 C2 Hit Count
                end
            end
            
            calc_check_convergence: begin
                //檢查C1 & C2是否已達收斂
                if (old_c1_best_x == c1_best_x && old_c1_best_y == c1_best_y 
                && old_c2_best_x == c2_best_x && old_c2_best_y == c2_best_y) begin
                    //已達收斂，進入計算完成狀態
                    calc_state <= calc_finish;
                end
                else begin
                    //未達收斂，重新設定C1的初始值開始新一輪的迭代
                    calc_state <= calc_set_c1;
                    //跳回第一階段來重置
                end
            end

            calc_finish: begin
                //計算完成，設置calc_done訊號
                //開始賦值給output
                C1X <= c1_best_x;
                C1Y <= c1_best_y;
                C2X <= c2_best_x;
                C2Y <= c2_best_y;
                calc_done <= 1'b1;
                calc_state <= calc_idle; //重置計算狀態以便下次使用
            end

            default: begin
                calc_state <= calc_idle; //預設回到計算閒置狀態
            end
        endcase
    end
end


//=================================Arithmetic Engine =========================//
// 這邊用Blocking Assignment來實作組合邏輯的部分 當值改變的時候才會進來計算
// 不要忘記blocking 的賦值方式是用= not <=
always @(*) begin
    //計算distance的部分
    X_dis = X_MEM[mark_data]- index_X;
    Y_dis = Y_MEM[mark_data]- index_Y;
    
    //蠻重要的實用小技巧：透過補述的方式去取得絕對值(必會)
    //key: 任何數與0的XOR會得到原本的數字
    //     任何數與1的XOR會得到該數的補數，最後再加上1即可得到絕對值
    X_abs = (X_dis ^ {5{X_dis[4]}}) + X_dis[4];
    Y_abs = (Y_dis ^ {5{Y_dis[4]}}) + Y_dis[4];
end

//=================================Judgemnt Engine =========================//
//透過曼哈頓距離＋閾值去判斷是否命中
always @(*) begin 
    //仔細看這邊補足了單純曼哈頓距離考慮不到的條件，因為的範圍是圓
    if ((X_abs + Y_abs <= 4) | (X_abs == 2 && Y_abs ==3) | (X_abs ==3 && Y_abs ==2)) begin
        hit = 1'b1; //命中
    end
    else begin
        hit = 1'b0; //未命中
    end
end

endmodule


