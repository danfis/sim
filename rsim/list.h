/***
 * sim
 * ---------------------------------
 * Copyright (c)2011 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */



#ifndef __SIM_LIST_H__
#define __SIM_LIST_H__

#include <string.h>
#include "compiler.h"

/**
 * List
 * =====
 */
struct _sim_list_t {
    struct _sim_list_t *next, *prev;
};
typedef struct _sim_list_t sim_list_t;

/**
 * Simliar struct as sim_list_t but with extra member "mark" which can be
 * used for marking a particular struct.
 * In your code, you can put this struct into your own struct then use
 * simListMAsList() for iterating over list and finaly simListMFromList()
 * for backcast to this struct. (In other words retyping from sim_list_m_t
 * to sim_list_t is safe!).
 */
struct _sim_list_m_t {
    struct _sim_list_m_t *next, *prev;
    int mark;
};
typedef struct _sim_list_m_t sim_list_m_t;

/**
 * Get the struct for this entry.
 * @ptr:	the &sim_list_t pointer.
 * @type:	the type of the struct this is embedded in.
 * @member:	the name of the list_struct within the struct.
 */
#define simListEntry(ptr, type, member) \
    sim_container_of(ptr, type, member)

/**
 * Iterates over list.
 */
#define simListForEach(list, item) \
        for (item = (list)->next; \
             _sim_prefetch((item)->next), item != (list); \
             item = (item)->next)

/**
 * Iterates over list safe against remove of list entry
 */
#define simListForEachSafe(list, item, tmp) \
	    for (item = (list)->next, tmp = (item)->next; \
             item != (list); \
		     item = tmp, tmp = (item)->next)

/**
 * Iterates over list of given type.
 * @pos:	the type * to use as a loop cursor.
 * @head:	the head for your list.
 * @member:	the name of the list_struct within the struct.
 */
#define simListForEachEntry(head, postype, pos, member) \
	for (pos = simListEntry((head)->next, postype, member);	\
	     _sim_prefetch(pos->member.next), &pos->member != (head); 	\
	     pos = simListEntry(pos->member.next, postype, member))

/**
 * Iterates over list of given type safe against removal of list entry
 * @pos:	the type * to use as a loop cursor.
 * @n:		another type * to use as temporary storage
 * @head:	the head for your list.
 * @member:	the name of the list_struct within the struct.
 */
#define simListForEachEntrySafe(head, postype, pos, n, ntype, member)         \
    for (pos = simListEntry((head)->next, postype, member),             \
		 n = simListEntry(pos->member.next, postype, member);	\
	     &pos->member != (head); 					\
	     pos = n, n = simListEntry(n->member.next, ntype, member))


/**
 * Functions
 * ----------
 */

/**
 * Initialize list.
 */
_sim_inline void simListInit(sim_list_t *l);

/**
 * Returns next element in list. If called on head first element is
 * returned.
 */
_sim_inline sim_list_t *simListNext(sim_list_t *l);

/**
 * Returns previous element in list. If called on head last element is
 * returned.
 */
_sim_inline sim_list_t *simListPrev(sim_list_t *l);

/**
 * Returns true if list is empty.
 */
_sim_inline int simListEmpty(const sim_list_t *head);

/**
 * Appends item to end of the list l.
 */
_sim_inline void simListAppend(sim_list_t *l, sim_list_t *item);

/**
 * Prepends item before first item in list.
 */
_sim_inline void simListPrepend(sim_list_t *l, sim_list_t *item);

/**
 * Removes item from list.
 */
_sim_inline void simListDel(sim_list_t *item);


/**
 * Returns number of items in list - this takes O(n).
 */
_sim_inline size_t simListSize(const sim_list_t *head);



/**
 * Retypes given "M" list struct to regular list struct.
 */
_sim_inline sim_list_t *simListMAsList(sim_list_m_t *l);

/**
 * Opposite to simListMAsList().
 */
_sim_inline sim_list_m_t *simListMFromList(sim_list_t *l);




///
/// INLINES:
///

_sim_inline void simListInit(sim_list_t *l)
{
    l->next = l;
    l->prev = l;
}

_sim_inline sim_list_t *simListNext(sim_list_t *l)
{
    return l->next;
}

_sim_inline sim_list_t *simListPrev(sim_list_t *l)
{
    return l->prev;
}

_sim_inline int simListEmpty(const sim_list_t *head)
{
    return head->next == head;
}

_sim_inline void simListAppend(sim_list_t *l, sim_list_t *new)
{
    new->prev = l->prev;
    new->next = l;
    l->prev->next = new;
    l->prev = new;
}

_sim_inline void simListPrepend(sim_list_t *l, sim_list_t *new)
{
    new->next = l->next;
    new->prev = l;
    l->next->prev = new;
    l->next = new;
}

_sim_inline void simListDel(sim_list_t *item)
{
    item->next->prev = item->prev;
    item->prev->next = item->next;
    item->next = item;
    item->prev = item;
}

_sim_inline size_t simListSize(const sim_list_t *head)
{
    sim_list_t *item;
    size_t size = 0;

    simListForEach(head, item){
        size++;
    }

    return size;
}

_sim_inline sim_list_t *simListMAsList(sim_list_m_t *l)
{
    return (sim_list_t *)l;
}

_sim_inline sim_list_m_t *simListMFromList(sim_list_t *l)
{
    return (sim_list_m_t *)l;
}

#endif /* __SIM_LIST_H__ */
