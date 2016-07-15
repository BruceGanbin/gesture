#ifndef __DOUBLE_LINK_H
#define __DOUBLE_LINK_H

#define NULL                         0

typedef struct NODE
{
    struct NODE *prv;
    struct NODE *next;
}Node,*pNode;

void link_init(pNode head);

void link_add_tail(pNode head ,pNode node);
void link_add_head(pNode head ,pNode node);

pNode link_rm_tail(pNode head);
pNode link_rm_head(pNode head);
pNode link_rm_node(pNode head);

unsigned int get_length_list(pNode);
pNode link_add_list(pNode phead, unsigned int pos ,pNode node);
pNode link_del_list(pNode phead, unsigned int pos);

#define list_entry(node,type,member)          \
    ((type *)((char *)(node)-(unsigned long)(&((type *)0)->member)))

void test_link(void);

#endif
