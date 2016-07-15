#include "link.h"

void link_init(pNode head)
{
    if(head == 0)
        return;
    head->prv  = head;
    head->next = head;
}

// ring double link
void link_node_add_head(pNode head ,pNode node)
{
    pNode tail;
    if(head == 0 || node == 0)
        return;
    tail = head->prv;
    head->prv = node;
    tail->next = node;
    node->prv = tail;
    node->next = head;
}

void link_node_add_tail(pNode head ,pNode node)
{
    if (head == 0 || node == 0)
        return;
    head->next = node;
    node->next = NULL;
    node->prv = head;
}

pNode link_node_rm_head(pNode head)
{
    pNode tail;
    if(head == 0)
        return 0;
    if(head->next == head || head->prv == head || head->prv == 0)
        return 0;

    tail = head->prv;
    head->prv = tail->prv;
    tail->prv->next = head;
    return tail;
}

pNode link_node_rm_tail(pNode head)
{
    pNode tail;
    if (head == 0)
        return 0;
    if(head->next == head || head->prv == head || head->next == 0)
        return 0;
    tail = head->next;
    head->next = tail->next;
    tail->next->prv = head;
    return tail;
}

pNode link_rm_node(pNode head)
{
    pNode tail,first;
    if (head == 0)
        return 0;
    first = head->prv;
    tail = head->next;
    if(tail != 0)
        tail->prv = first;
    if(first != 0)
        first->next = tail;
    return tail;
}

// stack double link
unsigned int get_length_list(pNode pHead)
{
    unsigned int len = 0;
    pNode pt = pHead;
    while(pt != NULL)
    {
        len++;
        pt = pt->next;
    }
    return len;
}

pNode link_find_head(pNode node)
{
    while(node->prv)
    {
        node = node->prv;
    }
    return node;
}

pNode link_add_list(pNode head,unsigned int pos,pNode node)
{
    pNode phead;
    phead = head;
    if(pos == 0)
    {
        return head;
    }

    while(pos > 0 && pos <= get_length_list(head))
    {
        pos--;

        if(phead->next == NULL)
            break;
        phead = phead->next;
    }
    link_node_add_tail(phead,node);
    return link_find_head(head);
}

pNode link_del_list(pNode head, unsigned int pos)
{
    pNode phead;
    phead = head;
    while((pos-1)>0 && pos<get_length_list(head)+2)
    {
        pos--;
        if(phead->next == NULL)
            break;
        phead = phead->next;
    }
    if(pos == 1) head = head->next; //remove tail node

    link_rm_node(phead);
    return link_find_head(head);
}


//--------------------test-------------------
#ifdef LINK_TEST
typedef struct
{
    Node link;
    unsigned char Data;
}Link_Data;

void test_link(void)
{
    Link_Data lk_da1;
    Link_Data lk_da2;
    Link_Data lk_da3;
    Link_Data lk_da4;
    Link_Data lk_da5;
    pNode  pHead = 0;
    unsigned char len;
    Link_Data *temp = 0;

    lk_da1.Data=1;
    lk_da2.Data=2;
    lk_da3.Data=3;
    lk_da4.Data=4;
    lk_da5.Data=5;

    len = get_length_list(pHead);
    pHead = &lk_da1.link;
    len = get_length_list(pHead);
    link_add_list(pHead,1,&lk_da2.link);
    len = get_length_list(pHead);
    link_add_list(pHead,2,&lk_da3.link);
    len = get_length_list(pHead);
    link_add_list(pHead,3,&lk_da4.link);
    len = get_length_list(pHead);
    link_add_list(pHead,4,&lk_da5.link);
    len = get_length_list(pHead);
    len = get_length_list(pHead);
    pHead = link_del_list(pHead,5);
    len = get_length_list(pHead);
    pHead = link_del_list(pHead,1);
    len = get_length_list(pHead);

    temp = list_entry(pHead->next->next,Link_Data,link);
    temp->Data++;
}
#endif
