#include <assert.h>
#include "../Core/Src/app_adc.c"
uint32_t mock_irq_mask=0;
int run=0;
int hspi2,anniu_zhuangtai[3][2],duty,chusaidaojuli,chusaidao;
float jiaoduz,jiaosuderror,jiaosuderrorl,jiaosudutobe;
void MX_SPI2_Init(void) {}

static void setup(int count)
{
    memset(anniu_zhuangtai,0,sizeof(anniu_zhuangtai));
    memset(adc_snapshot,0,sizeof(adc_snapshot));
    memset(adc_error_average,0,sizeof(adc_error_average));
    for (int i=0;i<14;i++) {adc_beijing[i]=0;adc_xian[i]=100;}
    calibrated=1;jiaozhun=0;adc_snapshot_active=0;jiaoduz=0;
    turn_count=count;turn_in_bend=0;turn_entry_cnt=0;turn_exit_cnt=0;
    turn_forced_flag=0;turn_entry_ms=1;lost_line_cnt=0;
    app_turn_table_set_enabled(0);
}
static void frame(int index)
{
    memset(adc_snapshot[0],0,sizeof(adc_snapshot[0]));
    if (index>=0) adc_snapshot[0][index]=100;
    app_adc_get();
}
int main(void)
{
    assert(turn_table_enabled==0); /* boot default */
    int numbers[20],exits[20];float angles[20];
    memcpy(numbers,turn_slot_num,sizeof(numbers));
    memcpy(exits,turn_slot_exit,sizeof(exits));
    memcpy(angles,turn_slot_angle,sizeof(angles));
    setup(6);frame(0);frame(0);
    assert(turn_count==7 && !turn_forced_flag); /* off still counts */
    setup(6);app_turn_table_set_enabled(1);frame(0);frame(0);
    assert(turn_count==7 && turn_forced_flag && forced_target_angle==-75);
    assert(adc_error_average[9]<-1); /* actual forced left-turn contribution */
    app_turn_table_set_enabled(0);
    assert(!turn_forced_flag && turn_count==7 && current_exit_ms==5);
    frame(6);
    assert(fabsf(adc_error_average[9])<.1f); /* actual sensor scan resumes */
    /* Re-enabling mid-bend must not restart that forced turn. */
    app_turn_table_set_enabled(1);frame(0);assert(!turn_forced_flag && turn_count==7);
    app_turn_table_set_enabled(0);
    for (int i=0;i<5;i++)frame(6);
    assert(!turn_in_bend);
    app_turn_table_set_enabled(1);frame(0);
    assert(turn_count==8 && turn_forced_flag && forced_target_angle==-25);
    app_turn_table_set_enabled(0);
    assert(memcmp(numbers,turn_slot_num,sizeof(numbers))==0);
    assert(memcmp(exits,turn_slot_exit,sizeof(exits))==0);
    assert(memcmp(angles,turn_slot_angle,sizeof(angles))==0);
    mock_irq_mask=1;app_turn_table_set_enabled(1);assert(mock_irq_mask==1);
    app_turn_table_set_enabled(0);assert(mock_irq_mask==1);
    mock_irq_mask=0;app_turn_table_set_enabled(1);assert(mock_irq_mask==0);
    run=1;app_turn_table_set_enabled(0);
    assert(!app_turn_table_set_enabled(1) && turn_table_enabled==0);
    assert(app_turn_table_set_enabled(0));run=0;
    puts("PASS: actual ADC state machine, independent switch, mid-turn cancel, next-bend enable, table preservation and IRQ mask restoration.");
}
