#include "multi_turn_store.h"
#include <string.h>
#include <math.h>
#define MAGIC 0x38544D53U
#define COMMIT 0x54494D43U
static void put32(uint8_t *p,uint32_t v)
{ unsigned i; for(i=0;i<4;i++) p[i]=(uint8_t)(v>>(8*i)); }
static uint32_t get32(const uint8_t *p)
{ return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static void put_float(uint8_t *p,float v)
{ uint32_t u; memcpy(&u,&v,4); put32(p,u); }
static float get_float(const uint8_t *p)
{ float f; uint32_t u=get32(p); memcpy(&f,&u,4); return f; }
/* CRC-32/ISO-HDLC（反射多项式 0xEDB88320），与记录正文 252 字节一致。 */
uint32_t MT_CRC32(const uint8_t *data,uint32_t size)
{
    uint32_t crc=0xffffffffU; unsigned j;
    while(size--) { crc^=*data++; for(j=0;j<8;j++) crc=(crc>>1)^((0U-(crc&1U))&0xedb88320U); }
    return ~crc;
}
/* 按 V8 布局编码一条 288 字节记录（含提交块），其余字节填 0xFF。 */
void MT_Encode(const MT_Record *r,uint8_t *b)
{
    unsigned i,j; uint64_t u;
    memset(b,0xff,MT_RECORD_SIZE);
    put32(b,MAGIC); b[4]=MT_FORMAT; b[5]=0; b[6]=0; b[7]=1;
    put32(b+8,r->seq); put32(b+12,r->config_id); put32(b+16,r->flags);
    put32(b+20,r->slave_valid_mask); put32(b+24,r->master_valid_mask); put32(b+28,r->source_protocol_id);
    for(i=0;i<7;i++) { put_float(b+32+4*i,r->slave_logical_pos[i]); put_float(b+60+4*i,r->slave_anchor[i]); }
    for(i=0;i<4;i++) put_float(b+88+4*i,r->master_wire_float[i]);
    for(i=0;i<2;i++) put32(b+104+4*i,(uint32_t)r->master_wire_int[i]);
    memcpy(b+112,r->master_payload,30);
    for(i=0;i<6;i++) {
        memcpy(&u,&r->master_anchor[i],8);
        for(j=0;j<8;j++) b[144+8*i+j]=(uint8_t)(u>>(8*j));
    }
    put32(b+192,r->mapped_axis_mask); put32(b+196,r->source_session);
    put32(b+200,r->master_session_flags); put32(b+204,r->calibration_state);
    put32(b+252,MT_CRC32(b,252));
    put32(b+256,COMMIT); put32(b+260,r->seq); put32(b+264,get32(b+252)); put32(b+268,~COMMIT);
}
static bool committed(const uint8_t *b)
{
    unsigned i;
    if(get32(b)!=MAGIC || b[6]!=0 || b[7]!=1 ||
       get32(b+256)!=COMMIT || get32(b+260)!=get32(b+8) ||
       get32(b+264)!=get32(b+252) || get32(b+268)!=~COMMIT ||
       MT_CRC32(b,252)!=get32(b+252)) return false;
    for(i=272;i<288;i++) if(b[i]!=255) return false;
    return true;
}
static bool sane(const MT_Record *r)
{
    unsigned i;
    if((r->slave_valid_mask&~127U)||(r->master_valid_mask&~63U)||
       (r->mapped_axis_mask&~127U)) return false;
    for(i=0;i<7;i++) if((r->slave_valid_mask&(1U<<i)) &&
        (!isfinite(r->slave_logical_pos[i]) || !isfinite(r->slave_anchor[i]))) return false;
    for(i=0;i<6;i++) if((r->master_valid_mask&(1U<<i)) &&
        (!isfinite(r->master_anchor[i]) || (i<4&&!isfinite(r->master_wire_float[i])))) return false;
    return true;
}
/* 校验并解析一条记录；任何字段越界或校验失败都返回 false。 */
bool MT_Decode(const uint8_t *b,MT_Record *r)
{
    unsigned i,j; uint64_t u;
    if(!committed(b) || b[4]!=MT_FORMAT || b[5]!=0) return false;
    memset(r,0,sizeof(*r));
    r->seq=get32(b+8); r->config_id=get32(b+12); r->flags=get32(b+16);
    r->slave_valid_mask=get32(b+20); r->master_valid_mask=get32(b+24); r->source_protocol_id=get32(b+28);
    for(i=0;i<7;i++) { r->slave_logical_pos[i]=get_float(b+32+4*i); r->slave_anchor[i]=get_float(b+60+4*i); }
    for(i=0;i<4;i++) r->master_wire_float[i]=get_float(b+88+4*i);
    for(i=0;i<2;i++) { uint32_t v=get32(b+104+4*i); memcpy(&r->master_wire_int[i],&v,4); }
    memcpy(r->master_payload,b+112,30);
    for(i=0;i<6;i++) { u=0; for(j=0;j<8;j++) u|=(uint64_t)b[144+8*i+j]<<(8*j); memcpy(&r->master_anchor[i],&u,8); }
    r->mapped_axis_mask=get32(b+192); r->source_session=get32(b+196);
    r->master_session_flags=get32(b+200); r->calibration_state=get32(b+204);
    return sane(r);
}
static bool blank(const uint8_t *p,unsigned n)
{ while(n--) if(*p++!=255) return false; return true; }
/* 扫描两个日志扇区，选出最新记录并定位各自的下一空槽。 */
MT_Result MT_Scan(MT_Store *s,const MT_Backend *io,uint32_t config)
{
    unsigned sector,slot; uint8_t b[288],latest[288]; uint32_t seq=0;
    if(!s||!io||!io->read||!io->program||!io->erase||!io->maintenance) return MT_INVALID;
    memset(s,0,sizeof(*s)); s->io=*io; s->config_id=config;
    for(sector=0;sector<2;sector++) for(slot=0;slot<MT_SLOTS;slot++) {
        if(!io->read(io->ctx,sector*MT_SECTOR_SIZE+slot*MT_RECORD_SIZE,b,288)) return MT_IO;
        if(!blank(b,288)) s->next[sector]=slot+1; /* 撕裂的槽绝不原地重写。 */
        if(committed(b)) {
            uint32_t candidate=get32(b+8),delta=candidate-seq;
            if(s->found && (delta==0x80000000U || (delta==0 && memcmp(b,latest,288)))) return MT_INVALID;
            if(!s->found || (delta!=0 && delta<0x80000000U)) {
                s->found=true; s->active=sector; seq=candidate; memcpy(latest,b,288);
            }
        }
    }
    if(!s->found) { s->scanned=true; return MT_EMPTY; }
    /* 最新的不兼容记录会阻挡恢复，绝不回退去复活旧的标定。 */
    if(!MT_Decode(latest,&s->latest)||s->latest.config_id!=config) return MT_INCOMPATIBLE;
    s->scanned=true; return MT_OK;
}
/* 写入一条新记录：先正文后提交块；满槽时按需换区，旧区保留。 */
MT_Result MT_Append(MT_Store *s,const MT_Record *input,bool allow_erase)
{
    MT_Record r; uint8_t b[288],verify[288]; unsigned target,slot,i;
    uint32_t addr;
    if(!s||!s->scanned||!input||input->config_id!=s->config_id||!sane(input)) return MT_INVALID;
    if(!s->io.maintenance(s->io.ctx)) return MT_LOCKED;
    r=*input; r.seq=s->found?s->latest.seq+1:1; target=s->active;
    if(s->next[target]>=MT_SLOTS) {
        target^=1;
        if(!allow_erase) return MT_FULL;
        /* 在新记录提交成功之前，旧的活跃扇区保持完整。 */
        if(!s->io.maintenance(s->io.ctx)||!s->io.erase(s->io.ctx,target)) { s->scanned=false; return MT_IO; }
        s->next[target]=0;
    }
    slot=s->next[target]++; addr=target*MT_SECTOR_SIZE+slot*MT_RECORD_SIZE;
    if(!s->io.read(s->io.ctx,addr,verify,288)||!blank(verify,288)) { s->scanned=false; return MT_IO; }
    MT_Encode(&r,b);
    for(i=0;i<256;i+=32)
        if(!s->io.maintenance(s->io.ctx)||!s->io.program(s->io.ctx,addr+i,b+i)) { s->scanned=false; return MT_IO; }
    if(!s->io.read(s->io.ctx,addr,verify,256)||memcmp(b,verify,256)) { s->scanned=false; return MT_IO; }
    if(!s->io.maintenance(s->io.ctx)||!s->io.program(s->io.ctx,addr+256,b+256) ||
       !s->io.read(s->io.ctx,addr,verify,288)||memcmp(b,verify,288)||!MT_Decode(verify,&r)) {
        s->scanned=false; return MT_IO;
    }
    s->latest=r; s->found=true; s->active=target;
    return MT_OK;
}
